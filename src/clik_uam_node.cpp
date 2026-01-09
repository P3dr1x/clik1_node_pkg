#include "clik1_node_pkg/clik_uam_node.hpp"

// Standard library (implementation-only)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <functional>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/centroidal.hpp" // Centroidal Momentum Matrix (Ag)
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/spatial/explog.hpp" // per log3 su SO(3)
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace {
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return S;
}
} // namespace


ClikUamNode::ClikUamNode() : Node("clik_uam_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Nodo clik_uam_node avviato.");

    this->declare_parameter<bool>("use_gazebo_pose", true);
    this->get_parameter("use_gazebo_pose", use_gazebo_pose_);

    this->declare_parameter<bool>("use_gz_odom", true);
    use_gz_odom_ = this->get_parameter("use_gz_odom").as_bool();

    // Parametri per instradare i topic e la modalità reale/simulata
    this->declare_parameter<std::string>("robot_name", "mobile_wx250s");
    this->declare_parameter<bool>("real_system", false);
    robot_name_ = this->get_parameter("robot_name").as_string();
    real_system_ = this->get_parameter("real_system").as_bool();

    // Carica il modello URDF
    // NOTA: il percorso del file URDF potrebbe dover essere reso un parametro
    const auto pkg_share = ament_index_cpp::get_package_share_directory("clik1_node_pkg");
    const std::string urdf_filename = pkg_share + "/model/t960a.urdf";
    
    RCLCPP_INFO(this->get_logger(), "Caricamento modello URDF da: %s", urdf_filename.c_str());
    
    try {
        pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model_);
        data_ = pinocchio::Data(model_);
        
        RCLCPP_INFO(this->get_logger(), "Modello URDF caricato con successo.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento del modello URDF: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    if (!model_.existFrame("mobile_wx250s/ee_gripper_link")) {
        RCLCPP_ERROR(this->get_logger(), "Frame 'mobile_wx250s/ee_gripper_link' assente nel modello.");
        rclcpp::shutdown();
        return;
    }
    ee_frame_id_ = model_.getFrameId("mobile_wx250s/ee_gripper_link");
    RCLCPP_INFO(this->get_logger(), "End-effector frame ID: %d", static_cast<int>(ee_frame_id_));

    // Giunti controllati dal controller (gruppo "arm").
    // Devono essere definiti PRIMA del precompute degli indici nel modello manipolatore.
    arm_joints_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};

    // Frame base braccio (punto di connessione O)
    if (!model_.existFrame("mobile_wx250s/base_link")) {
        RCLCPP_ERROR(this->get_logger(), "Frame 'mobile_wx250s/base_link' assente nel modello.");
        rclcpp::shutdown();
        return;
    }
    arm_base_frame_id_full_ = model_.getFrameId("mobile_wx250s/base_link");


    // Carica modello del SOLO manipolatore (FreeFlyer + braccio) per il task di momento
    {
        const std::string urdf_man_filename = pkg_share + "/model/wx250s.urdf";
        RCLCPP_INFO(this->get_logger(), "Caricamento modello manipolatore URDF da: %s", urdf_man_filename.c_str());
        try {
            pinocchio::urdf::buildModel(urdf_man_filename, pinocchio::JointModelFreeFlyer(), model_man_);
            data_man_ = pinocchio::Data(model_man_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento del modello manipolatore URDF: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        if (!model_man_.existFrame("mobile_wx250s/ee_gripper_link")) {
            RCLCPP_ERROR(this->get_logger(), "Frame EE assente nel modello manipolatore: mobile_wx250s/ee_gripper_link");
            rclcpp::shutdown();
            return;
        }
        ee_frame_id_man_ = model_man_.getFrameId("mobile_wx250s/ee_gripper_link");

        // Massa totale manipolatore (per tau_g)
        m_man_tot_ = 0.0;
        for (size_t i = 1; i < model_man_.inertias.size(); ++i) {
            m_man_tot_ += static_cast<double>(model_man_.inertias[i].mass());
        }
        g_world_man_ = model_man_.gravity.linear();

        // Precalcolo indici q/v nel modello manipolatore per gli stessi arm_joints_
        const int n_arm = static_cast<int>(arm_joints_.size());
        idx_v_arm_man_.resize(n_arm);
        idx_q_arm_man_.resize(n_arm);
        for (int i = 0; i < n_arm; ++i) {
            const std::string &jname = arm_joints_[static_cast<size_t>(i)];
            if (!model_man_.existJointName(jname)) {
                RCLCPP_ERROR(this->get_logger(), "Joint '%s' non esiste nel modello manipolatore.", jname.c_str());
                rclcpp::shutdown();
                return;
            }
            const pinocchio::JointIndex jid = model_man_.getJointId(jname);
            const int idx_v = static_cast<int>(model_man_.joints[jid].idx_v());
            const int idx_q = static_cast<int>(model_man_.joints[jid].idx_q());
            const int nq_j = static_cast<int>(model_man_.joints[jid].nq());
            if (nq_j != 1) {
                RCLCPP_ERROR(this->get_logger(), "Joint '%s' nel modello manipolatore ha nq!=1 (nq=%d)", jname.c_str(), nq_j);
                rclcpp::shutdown();
                return;
            }
            idx_v_arm_man_[i] = idx_v;
            idx_q_arm_man_[i] = idx_q;
        }
    }

    // SOTTOSCRIZIONI STATO DRONE
    if (use_gazebo_pose_) {
        RCLCPP_INFO(this->get_logger(), "Utilizzo della posa da Gazebo (/world/default/dynamic_pose/info).");
        gazebo_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/world/default/dynamic_pose/info", 10, std::bind(&ClikUamNode::gazebo_pose_callback, this, std::placeholders::_1));
    } else {
        RCLCPP_INFO(this->get_logger(), "Utilizzo della posa dal Motion Capture (/t960a/pose - PoseStamped da natnet_ros2).");
        real_drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/t960a/pose", 10, std::bind(&ClikUamNode::real_drone_pose_callback, this, std::placeholders::_1));
    }

    // Sottoscrizione alla velocità/odometria del drone. Regole:
    // - Se use_gz_odom == false: usa /real_t960a_twist (da real_drone_vel_pub) e NON iscriversi a /model/t960a_0/odometry
    // - Se use_gz_odom == true e use_gazebo_pose_ == true: usa /model/t960a_0/odometry
    // - Se real_system_ == true: comunque iscriviti a /real_t960a_twist

    if (!use_gz_odom_ || real_system_ || !use_gazebo_pose_)
    {
        real_drone_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/real_t960a_twist", rclcpp::SensorDataQoS(),
            std::bind(&ClikUamNode::real_drone_twist_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Iscrizione a /real_t960a_twist (Twist FLU) [use_gz_odom=%d, real_system=%d, use_gazebo_pose=%d]",
                    use_gz_odom_, real_system_, use_gazebo_pose_);
    }

    if (use_gz_odom_ && use_gazebo_pose_)
    {
        gazebo_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/t960a_0/odometry", 10,
            std::bind(&ClikUamNode::gazebo_odometry_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Iscrizione a /model/t960a_0/odometry (nav_msgs/Odometry) [use_gz_odom=true, use_gazebo_pose=true]");
    }

    // Joint states: se reale, leggi da /<robot_name>/joint_states (xs_sdk); se SITL, da /joint_states (ros2_control)
    {
        std::string joint_states_topic;
        // if (real_system_) {
        //     joint_states_topic = "/" + robot_name_ + "/joint_states";
        // } else {
        joint_states_topic = "/joint_states";
        // }
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, 10, std::bind(&ClikUamNode::joint_state_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Mi sottoscrivo a %s", joint_states_topic.c_str());
    }

    // Publisher comandi: se reale, JointGroupCommand su /<robot_name>/commands/joint_group; se SITL, Float64MultiArray su /arm_controller/commands
    if (real_system_) {
        const std::string cmd_topic = "/commands/joint_group";
        joint_group_pub_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(cmd_topic, 10);
        RCLCPP_INFO(this->get_logger(), "real_system=true -> pubblicherò su %s (JointGroupCommand)", cmd_topic.c_str());
    } else {
        arm_controller_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "real_system=false -> pubblicherò su /arm_controller/commands (Float64MultiArray)");
    }

    // Pose corrente EE (rispetto al frame world)
    ee_world_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ee_world_pose", 10);

    // Subscription alla posa desiderata pubblicata dal planner (QoS latched)
    desired_ee_global_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/desired_ee_global_pose", rclcpp::QoS(10),  // QoS volatile: non riceve messaggi precedenti all'avvio
        std::bind(&ClikUamNode::desired_pose_callback, this, std::placeholders::_1));

    // Subscription alla velocità desiderata EE (default: zero se non presente)
    desired_ee_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/desired_ee_velocity", rclcpp::QoS(10),
        std::bind(&ClikUamNode::desired_velocity_callback, this, std::placeholders::_1));

    // Allocazioni
    q_.resize(model_.nq);
    q_.setZero();
    qd_.resize(model_.nv);
    qd_.setZero();
    v_gen_meas_.resize(model_.nv);
    v_gen_meas_.setZero();
    J_.resize(6, model_.nv);
    Jgen_.resize(6, model_.nv - 6);
    desired_ee_velocity_vec_.resize(6);
    desired_ee_velocity_vec_.setZero();
    // Nota: arm_joints_ è già impostato sopra (prima del modello manipolatore).

    declare_parameter("k_err_x_", 20.0); // legacy (tenuto per compatibilità)
    declare_parameter("kp_pos", 20.0);
    declare_parameter("kp_ori", 20.0);
    declare_parameter("damping", 1e-4);   // damping per pseudoinversa (Tikhonov)
    declare_parameter("qp_lambda_reg", 1e-4);
    declare_parameter("qp_vel_max_default", 2.0);
    declare_parameter("qp_eps_ab", 1e-9);
    // Parametri opzionali per pesi (spalla, forearm_roll, wrist_rotate hanno peso maggiore).
    declare_parameter("shoulder_weight", 15.0);
    declare_parameter("forearm_weight", 25.0);
    declare_parameter("wrist_weight", 25.0);
    // Opzione per sfruttare ridondanza cinematica:
    // - true  -> Jext (jext) con task [EE lin; momento] (default)
    // - false -> pose-mom con due costi pesati (tracking posa EE 6D + momento)
    this->declare_parameter<bool>("redundant", true);
    redundant_ = this->get_parameter("redundant").as_bool();

    // Pesi per la formulazione pose-mom (usati solo con redundant=false)
    this->declare_parameter<double>("w_kin", 10.0);
    this->declare_parameter<double>("w_mom", 1.0);
    w_kin_ = this->get_parameter("w_kin").as_double();
    w_mom_ = this->get_parameter("w_mom").as_double();

    // Abilita/disabilita il termine basato sul momento totale h_UAM nel task di momento (solo redundant=true)
    this->declare_parameter<bool>("use_h_uam", false);
    use_h_uam_ = this->get_parameter("use_h_uam").as_bool();
    // Parametro per abilitare la null-space velocity qd_N = qd(k-1)
    this->declare_parameter<bool>("qd_N_prev", false);
    kp_pos_ = get_parameter("kp_pos").as_double();
    kp_ori_ = get_parameter("kp_ori").as_double();
    qp_lambda_reg_ = get_parameter("qp_lambda_reg").as_double();
    qp_vel_max_default_ = get_parameter("qp_vel_max_default").as_double();
    double shoulder_w = get_parameter("shoulder_weight").as_double();
    double forearm_w = get_parameter("forearm_weight").as_double();
    double wrist_w = get_parameter("wrist_weight").as_double();
    W_diag_.resize(arm_joints_.size());
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
        const std::string &jn = arm_joints_[i];
        if (jn == "shoulder") W_diag_[static_cast<Eigen::Index>(i)] = shoulder_w;
        else if (jn == "forearm_roll") W_diag_[static_cast<Eigen::Index>(i)] = forearm_w;
        else if (jn == "wrist_rotate") W_diag_[static_cast<Eigen::Index>(i)] = wrist_w;
        else W_diag_[static_cast<Eigen::Index>(i)] = 1.0;
    }
    // Precalcolo una tantum: indici di velocità e posizione per i giunti del braccio
    {
        const int n_arm = static_cast<int>(arm_joints_.size());
        idx_v_arm_.resize(n_arm);
        idx_q_arm_.resize(n_arm);
        for (int i = 0; i < n_arm; ++i) {
            const std::string &jname = arm_joints_[static_cast<size_t>(i)];
            if (!model_.existJointName(jname)) {
                RCLCPP_ERROR(this->get_logger(), "Joint '%s' non esiste nel modello.", jname.c_str());
                rclcpp::shutdown();
                return;
            }
            const pinocchio::JointIndex jid = model_.getJointId(jname);
            const int idx_v = static_cast<int>(model_.joints[jid].idx_v());
            const int idx_q = static_cast<int>(model_.joints[jid].idx_q());
            const int nq_j = static_cast<int>(model_.joints[jid].nq());
            if (idx_v < 0 || idx_v >= static_cast<int>(model_.nv)) {
                RCLCPP_ERROR(this->get_logger(), "idx_v fuori range per joint '%s' (idx_v=%d, nv=%d)",
                             jname.c_str(), idx_v, static_cast<int>(model_.nv));
                rclcpp::shutdown();
                return;
            }
            idx_v_arm_[i] = idx_v;
            if (nq_j != 1 || idx_q < 0 || idx_q >= static_cast<int>(model_.nq)) {
                RCLCPP_ERROR(this->get_logger(), "idx_q fuori range o nq_j!=1 per joint '%s' (idx_q=%d, nq=%d)",
                             jname.c_str(), idx_q, static_cast<int>(model_.nq));
                rclcpp::shutdown();
                return;
            }
            idx_q_arm_[i] = idx_q;
        }
    }

    // Pre-estrazione limiti di posizione/velocità (ordine arm_joints_)
    {
        const int n_arm = static_cast<int>(arm_joints_.size());
        q_lower_arm_.resize(n_arm);
        q_upper_arm_.resize(n_arm);
        v_limit_arm_.resize(n_arm);
        q_lower_arm_.setZero();
        q_upper_arm_.setZero();
        v_limit_arm_.setZero();

        have_position_limits_ = (model_.lowerPositionLimit.size() == static_cast<Eigen::Index>(model_.nq)) &&
                               (model_.upperPositionLimit.size() == static_cast<Eigen::Index>(model_.nq));
        have_velocity_limits_ = (model_.velocityLimit.size() == static_cast<Eigen::Index>(model_.nv));

        for (int i = 0; i < n_arm; ++i) {
            const int iq = idx_q_arm_[i];
            const int iv = idx_v_arm_[i];
            if (have_position_limits_) {
                q_lower_arm_[i] = model_.lowerPositionLimit[static_cast<Eigen::Index>(iq)];
                q_upper_arm_[i] = model_.upperPositionLimit[static_cast<Eigen::Index>(iq)];
            } else {
                q_lower_arm_[i] = -std::numeric_limits<double>::infinity();
                q_upper_arm_[i] = +std::numeric_limits<double>::infinity();
            }
            if (have_velocity_limits_) {
                v_limit_arm_[i] = model_.velocityLimit[static_cast<Eigen::Index>(iv)];
            } else {
                v_limit_arm_[i] = 0.0;
            }
        }
    }

    // Setup QP (matrici con sparsità fissa). Il solver viene inizializzato una sola volta.
    {
        qp_n_ = static_cast<int>(arm_joints_.size());

        // Work buffers
        qp_gradient_.resize(qp_n_);
        qp_l_.resize(qp_n_);
        qp_u_.resize(qp_n_);
        qp_solution_.resize(qp_n_);
        qp_solution_.setZero();
        qp_q_arm_meas_.resize(qp_n_);
        qp_q_arm_meas_.setZero();

        qp_P_dense_.resize(qp_n_, qp_n_);
        qp_P_dense_.setZero();
        qp_J_task_.resize(6, qp_n_);
        qp_J_task_.setZero();
        qp_v_task_.resize(6);
        qp_v_task_.setZero();

        // A = I (box constraints)
        qp_A_.resize(qp_n_, qp_n_);
        qp_A_.setIdentity();
        qp_A_.makeCompressed();

        // Hessian sparsity: full upper-triangular pattern
        qp_hessian_.resize(qp_n_, qp_n_);
        qp_hessian_.reserve(Eigen::Index(qp_n_ * (qp_n_ + 1) / 2));
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(static_cast<size_t>(qp_n_ * (qp_n_ + 1) / 2));
        for (int j = 0; j < qp_n_; ++j) {
            for (int i = 0; i <= j; ++i) {
                triplets.emplace_back(i, j, (i == j) ? qp_lambda_reg_ : 0.0);
            }
        }
        qp_hessian_.setFromTriplets(triplets.begin(), triplets.end());
        qp_hessian_.makeCompressed();

        // OSQP-Eigen data
        qp_solver_.settings()->setVerbosity(false);
        qp_solver_.settings()->setWarmStart(true);
        qp_solver_.settings()->setPolish(false);
        qp_solver_.data()->setNumberOfVariables(qp_n_);
        qp_solver_.data()->setNumberOfConstraints(qp_n_);
        qp_solver_.data()->setLinearConstraintsMatrix(qp_A_);

        // (H, g, l, u) verranno settati al primo ciclo utile quando dt e q sono validi
        qp_initialized_ = false;
    }

    // Timer di update() controllo parametrico
    this->declare_parameter<double>("control_rate_hz", 100.0);
    double rate_hz = this->get_parameter("control_rate_hz").as_double();
    rate_hz = std::max(1.0, rate_hz); // salvaguardia: minimo 1 Hz
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    control_timer_ = this->create_wall_timer(period_ns, std::bind(&ClikUamNode::update, this));
    RCLCPP_INFO(this->get_logger(), "control_rate_hz=%.2f Hz (periodo=%.3f ms)", rate_hz, 1000.0 / rate_hz);
    last_update_time_ = this->now();
    this->declare_parameter<double>("desired_timeout_sec", 0.5);
    desired_timeout_sec_ = this->get_parameter("desired_timeout_sec").as_double();
}

void ClikUamNode::desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    desired_ee_pose_world_ = *msg;
    desired_ee_pose_world_ready_ = true;
    last_desired_msg_time_ = this->now();
    have_desired_msg_ = true;
    //RCLCPP_INFO(this->get_logger(), "Nuova posa desiderata ricevuta: x=%.3f y=%.3f z=%.3f", msg->position.x, msg->position.y, msg->position.z);
}

void ClikUamNode::desired_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    desired_ee_velocity_world_ = *msg;
    desired_ee_velocity_ready_ = true; // indica che siamo in una fase di tracking
    //RCLCPP_INFO(this->get_logger(), "Nuova velocità desiderata ricevuta: x=%.3f y=%.3f z=%.3f", msg->linear.x, msg->linear.y, msg->linear.z);
    last_desired_msg_time_ = this->now();
    have_desired_msg_ = true;

}

void ClikUamNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    // Conversione da NED (PX4) a FLU (Forward, Left, Up)
    // NED: x=N (Forward), y=E (Right), z=D (Down)
    // FLU: x=Forward -> x_ned, y=Left -> -y_ned, z=Up -> -z_ned
    vehicle_local_position_ = *msg;             // copia originale
    vehicle_local_position_.x = msg->y;         // Forward
    vehicle_local_position_.y = msg->x;         // Left
    vehicle_local_position_.z = -msg->z;        // Up

    has_vehicle_local_position_ = true;
}

void ClikUamNode::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    // Conversione da FRD (PX4) a FLU (ROS2)
    // Eigen::Quaterniond frd_quat(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    // FRD (Forward, Right, Down) -> FLU (Forward, Left, Up)
    // La conversione consiste nell'invertire gli assi Y e Z  
    // Eigen::Quaterniond flu_quat(frd_quat.w(), frd_quat.x(), -frd_quat.y(), -frd_quat.z());
    Eigen::Quaterniond flu_quat(msg->q[0], msg->q[1], -msg->q[2], -msg->q[3]);


    vehicle_attitude_ = *msg; // Copio il messaggio originale
    // Sovrascrivo con il quaternione convertito
    vehicle_attitude_.q[0] = flu_quat.w();
    vehicle_attitude_.q[1] = flu_quat.x();
    vehicle_attitude_.q[2] = flu_quat.y();
    vehicle_attitude_.q[3] = flu_quat.z();

    has_vehicle_attitude_ = true;
}

void ClikUamNode::real_drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Assumiamo che /t960a/pose sia in frame world FLU (come configurato in natnet_ros2)
    const auto& p = msg->pose.position;
    const auto& o = msg->pose.orientation;
    vehicle_local_position_.x = p.x;
    vehicle_local_position_.y = p.y;
    vehicle_local_position_.z = p.z;

    vehicle_attitude_.q[0] = o.w;
    vehicle_attitude_.q[1] = o.x;
    vehicle_attitude_.q[2] = o.y;
    vehicle_attitude_.q[3] = o.z;
    has_vehicle_local_position_ = true;
    has_vehicle_attitude_ = true;
}

void ClikUamNode::gazebo_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (!msg->poses.empty())
    {
        const auto& pose = msg->poses[0];
        vehicle_local_position_.x = pose.position.x;
        vehicle_local_position_.y = pose.position.y;
        vehicle_local_position_.z = pose.position.z;

        vehicle_attitude_.q[0] = pose.orientation.w;
        vehicle_attitude_.q[1] = pose.orientation.x;
        vehicle_attitude_.q[2] = pose.orientation.y;
        vehicle_attitude_.q[3] = pose.orientation.z;

        has_vehicle_local_position_ = true;
        has_vehicle_attitude_ = true;
    }
}

void ClikUamNode::gazebo_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    gazebo_odom_ = *msg;
    has_gazebo_odom_ = true;
}

void ClikUamNode::real_drone_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Twist già espresso in:
    // - lineare: WORLD-FLU con heading iniziale rimosso (da real_drone_vel_pub)
    // - angolare: body FLU
    real_drone_twist_ = *msg;
    has_real_drone_twist_ = true;
}

void ClikUamNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Copia il messaggio
    current_joint_state_ = *msg;

    // Se il topic non fornisce le velocità o contiene NaN/Inf, stimale via differenze finite
    const bool names_ok = !current_joint_state_.name.empty();
    const bool pos_ok = !current_joint_state_.position.empty();
    const bool vel_absent = current_joint_state_.velocity.empty();
    bool vel_nonfinite = false;
    if (!vel_absent) {
        for (const auto &v : current_joint_state_.velocity) {
            if (!std::isfinite(v)) {
                vel_nonfinite = true;
                break;
            }
        }
    }

    if ((vel_absent || vel_nonfinite) && names_ok && pos_ok)
    {
        // Usa timestamp del messaggio se disponibile; altrimenti usa ora
        rclcpp::Time stamp = current_joint_state_.header.stamp;
        if (stamp.nanoseconds() == 0) {
            stamp = this->now();
        }

        std::vector<double> vel_est(current_joint_state_.name.size(), 0.0);
        double dt = 0.0;
        if (last_joint_state_time_.nanoseconds() != 0) {
            dt = (stamp - last_joint_state_time_).seconds();
        }

        if (dt > 1e-5) {
            for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
                const auto &jn = current_joint_state_.name[i];
                const double q_now = current_joint_state_.position[i];
                auto it = prev_joint_positions_.find(jn);
                if (it != prev_joint_positions_.end() && std::isfinite(q_now)) {
                    const double q_prev = it->second;
                    vel_est[i] = (q_now - q_prev) / dt;
                } else {
                    vel_est[i] = 0.0;
                }
            }
        } else {
            std::fill(vel_est.begin(), vel_est.end(), 0.0);
        }

        current_joint_state_.velocity = vel_est;

        for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
            prev_joint_positions_[current_joint_state_.name[i]] = current_joint_state_.position[i];
        }
        last_joint_state_time_ = stamp;
    } else {
        if (names_ok && pos_ok) {
            for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
                prev_joint_positions_[current_joint_state_.name[i]] = current_joint_state_.position[i];
            }
            rclcpp::Time stamp = current_joint_state_.header.stamp;
            if (stamp.nanoseconds() == 0) {
                stamp = this->now();
            }
            last_joint_state_time_ = stamp;
        }
    }

    has_current_joint_state_ = true;
}

void ClikUamNode::update()
{
    if (!desired_ee_pose_world_ready_ || !has_current_joint_state_ || current_joint_state_.name.empty() || !has_vehicle_local_position_ || !has_vehicle_attitude_)
    {
        if (!waiting_log_printed_) {
            RCLCPP_INFO(this->get_logger(), "In attesa di: posa_desiderata=%d joint_state=%d posa_drone=%d attitude_drone=%d",
                        desired_ee_pose_world_ready_, has_current_joint_state_, has_vehicle_local_position_, has_vehicle_attitude_);
            waiting_log_printed_ = true;
        }
        // last_update_time_ = this->now(); //Mantieni aggiornato il timestamp mentre si attende: evita un dt accumulato al primo ciclo utile

        return;
    }
    // appena tutti i dati disponibili, reset del flag
    if (waiting_log_printed_) {
        RCLCPP_INFO(this->get_logger(), "Dati pronti. Avvio controllo.");
        waiting_log_printed_ = false;
    }

    // dt del controllo (serve sia per il task di momento che per i vincoli discretizzati del QP)
    const rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    dt = std::clamp(dt, 1e-4, 0.02); // evita divisione per 0, max 20ms

    // 1. AGGIORNA STATO DEL ROBOT 
    // q_.setZero();

    q_[0] = vehicle_local_position_.x;
    q_[1] = vehicle_local_position_.y;
    q_[2] = vehicle_local_position_.z;
    q_[3] = vehicle_attitude_.q[1]; // x
    q_[4] = vehicle_attitude_.q[2]; // y
    q_[5] = vehicle_attitude_.q[3]; // z
    q_[6] = vehicle_attitude_.q[0]; // w

    // Velocità WORLD del drone
    Eigen::Vector3d vlin_base_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_base_world = Eigen::Vector3d::Zero();

    // Sorgenti disponibili:
    //  - Twist FLU pubblicato da real_drone_vel_pub su /real_t960a_twist
    //  - Odometria Gazebo (WORLD-FLU) su /model/t960a_0/odometry
    if (has_real_drone_twist_)
    {
        vlin_base_world = Eigen::Vector3d(
            real_drone_twist_.linear.x,
            real_drone_twist_.linear.y,
            real_drone_twist_.linear.z);
    }
    else if (use_gazebo_pose_ && has_gazebo_odom_)
    {
        vlin_base_world = Eigen::Vector3d(
            gazebo_odom_.twist.twist.linear.x,
            gazebo_odom_.twist.twist.linear.y,
            gazebo_odom_.twist.twist.linear.z);

        omega_base_world = Eigen::Vector3d(
            gazebo_odom_.twist.twist.angular.x,
            gazebo_odom_.twist.twist.angular.y,
            gazebo_odom_.twist.twist.angular.z);
    }

    // Converti WORLD -> LOCAL usando R^T per la parte lineare.
    // La parte angolare, nel caso reale, arriva già in body FLU da real_drone_vel_pub.
    Eigen::Quaterniond q_world_base(vehicle_attitude_.q[0], vehicle_attitude_.q[1], vehicle_attitude_.q[2], vehicle_attitude_.q[3]);
    Eigen::Matrix3d Rwb = q_world_base.normalized().toRotationMatrix();
    Eigen::Vector3d vlin_base_local = Rwb.transpose() * vlin_base_world;

    Eigen::Vector3d omega_base_local;
    if (has_real_drone_twist_)
    {
        omega_base_local = Eigen::Vector3d(
            real_drone_twist_.angular.x,
            real_drone_twist_.angular.y,
            real_drone_twist_.angular.z);
    }
    else
    {
        omega_base_local = Rwb.transpose() * omega_base_world;
    }

    // 2. LEGGI POSIZIONI GIUNTI BRACCIO
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
        // Cerca il giunto nel modello di Pinocchio e aggiorna q_
        const auto& joint_name = current_joint_state_.name[i];
        if (model_.existJointName(joint_name)) {
            pinocchio::JointIndex joint_idx = model_.getJointId(joint_name);
            int joint_idx_int = static_cast<int>(joint_idx);
            if (joint_idx_int > 1 && (joint_idx_int - 2 + 7) < model_.nq) {
                q_[joint_idx_int - 2 + 7] = current_joint_state_.position[i];
            }
        }
    }
    pinocchio::normalize(model_, q_);

    // Estrai velocità giunti dai JointState (se disponibili)
    qd_.setZero();
    if (!current_joint_state_.velocity.empty()) {
        for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
            const auto &jn = current_joint_state_.name[i];
            if (model_.existJointName(jn)) {
                pinocchio::JointIndex jidx = model_.getJointId(jn);
                int jidx_int = static_cast<int>(jidx);
                int col = jidx_int - 2;
                if (col >= 0 && (6 + col) < model_.nv && i < current_joint_state_.velocity.size()) {
                    qd_[6 + col] = current_joint_state_.velocity[i];
                }
            }
        }
    }

    // Generalized velocity misurata (usa velocità drone misurate + velocità giunti misurate)
    v_gen_meas_.setZero();
    v_gen_meas_.segment<3>(0) = vlin_base_local;
    v_gen_meas_.segment<3>(3) = omega_base_local;
    for (int k = 6; k < model_.nv; ++k) {
        v_gen_meas_[k] = qd_[k];
    }

    // Cinematica diretta per posa assoluta dell'end-effector
    pinocchio::forwardKinematics(model_, data_, q_, v_gen_meas_);
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3& ee_placement = data_.oMf[ee_frame_id_];

    // 3. CALCOLO JACOBIANI E CMM (Centroidal Momentum Matrix)

    // Jacobiano del frame in LOCAL_WORLD_ALIGNED
    // RIC: il vettore velocità desiderata ha convenzione [lin; ang]
    pinocchio::computeFrameJacobian(model_, data_, q_, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_);

    // Calcolo della Centroidal Momentum Matrix Ag(q)
    // Ag è 6 x nv e si partiziona in [Ab | Am], con Ab=Ag(:,0:5), Am=Ag(:,6:nv-1)
    pinocchio::computeCentroidalMap(model_, data_, q_);
    const Eigen::MatrixXd &Ag = data_.Ag; // 6 x nv
    Eigen::MatrixXd Ab = Ag.leftCols(6);                     // 6 x 6
    Eigen::MatrixXd Am = Ag.rightCols(model_.nv - 6);        // 6 x (nv-6)

    // Regolarizzazione Ab (come nello script Python): Ab_reg = Ab + eps * I
    const double eps_Ab = this->get_parameter("qp_eps_ab").as_double();
    Eigen::Matrix<double, 6, 6> Ab_reg = Ab;
    Ab_reg.diagonal().array() += eps_Ab;

    // Estrai i blocchi del Jacobiano dell'EE
    Eigen::MatrixXd J_b = J_.leftCols(6);
    Eigen::MatrixXd J_m = J_.rightCols(model_.nv - 6);

    // 4. CALCOLO JACOBIANO GENERALIZZATO tramite Ab, Am (Paper_MP Sec. 3.1)
    // Jgen = J_m - J_b * Ab^{-1} * Am
    Eigen::MatrixXd Ab_inv_Am = Ab_reg.ldlt().solve(Am); // risolve Ab_reg * X = Am
    Jgen_ = J_m - J_b * Ab_inv_Am;

    // Seleziona le colonne del Jacobiano generalizzato corrispondenti ai soli giunti del braccio
    const int n_arm = static_cast<int>(arm_joints_.size());
    Eigen::MatrixXd Jgen_arm(6, n_arm);
    for (int i = 0; i < n_arm; ++i) {
        const int idx_v = idx_v_arm_[i];
        const int col = idx_v - 6; // rimuovi i 6 DoF di base
        if (col >= 0 && col < Jgen_.cols()) {
            Jgen_arm.col(i) = Jgen_.col(col);
        } else {
            Jgen_arm.col(i).setZero();
            RCLCPP_WARN(this->get_logger(), "Colonna Jgen per joint %s fuori range (idx_v=%d col=%d)",
                        arm_joints_[static_cast<size_t>(i)].c_str(), idx_v, col);
        }
    }

    // ==== Task di momento manipolatore rispetto a O (mobile_wx250s/base_link) ====
    // Calcola J_mom = (A_KO,b^man * Ab^{-1} * Am + A_KO,m^man) selezionando solo i giunti arm_joints_
    Eigen::MatrixXd J_mom_arm(3, n_arm);
    J_mom_arm.setZero();
    Eigen::Vector3d v_mom_task = Eigen::Vector3d::Zero();
    Eigen::Vector3d h_uam_term = Eigen::Vector3d::Zero();

    {
        // Costruisci stato (q_man, v_man) del manipolatore imponendo la posa di O dal modello completo
        Eigen::VectorXd q_man(model_man_.nq);
        Eigen::VectorXd v_man(model_man_.nv);
        q_man.setZero();
        v_man.setZero();

        // Posa di O nel mondo dal modello completo
        const pinocchio::SE3 &T_w_O = data_.oMf[arm_base_frame_id_full_];
        q_man.segment<3>(0) = T_w_O.translation();
        Eigen::Quaterniond q_O(T_w_O.rotation());
        q_O.normalize();
        q_man[3] = q_O.x();
        q_man[4] = q_O.y();
        q_man[5] = q_O.z();
        q_man[6] = q_O.w();

        // Copia posizioni giunti braccio per nome (ordine arm_joints_)
        for (int i = 0; i < n_arm; ++i) {
            const int iq_full = idx_q_arm_[i];
            const int iq_man = idx_q_arm_man_[i];
            if (iq_full >= 0 && iq_full < q_.size() && iq_man >= 0 && iq_man < q_man.size()) {
                q_man[iq_man] = q_[iq_full];
            }
        }

        // Velocità base del braccio nel frame LOCAL (coerente con JointModelFreeFlyer)
        const pinocchio::Motion V_O_local = pinocchio::getFrameVelocity(model_, data_, arm_base_frame_id_full_, pinocchio::ReferenceFrame::LOCAL);
        v_man.segment<3>(0) = V_O_local.linear();
        v_man.segment<3>(3) = V_O_local.angular();

        // Copia velocità giunti braccio (ordine arm_joints_)
        for (int i = 0; i < n_arm; ++i) {
            const int iv_full = idx_v_arm_[i];
            const int iv_man = idx_v_arm_man_[i];
            if (iv_full >= 0 && iv_full < v_gen_meas_.size() && iv_man >= 0 && iv_man < v_man.size()) {
                v_man[iv_man] = v_gen_meas_[iv_full];
            }
        }

        // Centroidal map e momentum del manipolatore
        pinocchio::forwardKinematics(model_man_, data_man_, q_man, v_man);
        pinocchio::centerOfMass(model_man_, data_man_, q_man, v_man, false);
        pinocchio::computeCentroidalMap(model_man_, data_man_, q_man);
        pinocchio::computeCentroidalMomentum(model_man_, data_man_, q_man, v_man);

        const Eigen::MatrixXd &Ag_man = data_man_.Ag; // 6 x (6+n_arm_man)
        const Eigen::MatrixXd A_p_man = Ag_man.topRows(3);
        const Eigen::MatrixXd A_K_man = Ag_man.bottomRows(3);

        // Trasporto: A_KO = A_K + skew(GmO) * A_p
        const Eigen::Vector3d O_w = q_man.segment<3>(0);
        const Eigen::Vector3d Gm_w = data_man_.com[0];
        const Eigen::Vector3d GmO_w = (O_w - Gm_w);

        Eigen::MatrixXd A_KO_man = A_K_man + skew(GmO_w) * A_p_man; // 3 x nv_man
        Eigen::MatrixXd A_KO_b_man = A_KO_man.leftCols(6);          // 3 x 6

        // Seleziona solo le colonne (giunti arm_joints_) per A_KO_m
        Eigen::MatrixXd A_KO_m_arm(3, n_arm);
        for (int i = 0; i < n_arm; ++i) {
            const int iv_man = idx_v_arm_man_[i];
            if (iv_man >= 6 && iv_man < A_KO_man.cols()) {
                A_KO_m_arm.col(i) = A_KO_man.col(iv_man);
            } else {
                A_KO_m_arm.col(i).setZero();
            }
        }

        // Ab^{-1} * Am selezionato sulle colonne dei giunti arm
        Eigen::MatrixXd Ab_inv_Am_arm(6, n_arm);
        for (int i = 0; i < n_arm; ++i) {
            const int iv_full = idx_v_arm_[i];
            const int col = iv_full - 6;
            if (col >= 0 && col < Ab_inv_Am.cols()) {
                Ab_inv_Am_arm.col(i) = Ab_inv_Am.col(col);
            } else {
                Ab_inv_Am_arm.col(i).setZero();
            }
        }

        J_mom_arm.noalias() = A_KO_b_man * Ab_inv_Am_arm + A_KO_m_arm;

        // Momentum task RHS: K_O + dt*(v_O x p_man + tau_g)
        const Eigen::Vector3d p_man = data_man_.hg.linear();
        const Eigen::Vector3d K_Gm_man = data_man_.hg.angular();
        const Eigen::Vector3d K_O_man = K_Gm_man + GmO_w.cross(p_man);

        const pinocchio::Motion V_O_world = pinocchio::getFrameVelocity(model_, data_, arm_base_frame_id_full_, pinocchio::ReferenceFrame::WORLD);
        const Eigen::Vector3d v_O_world = V_O_world.linear();

        const Eigen::Vector3d Fg_man = m_man_tot_ * g_world_man_;
        const Eigen::Vector3d tau_g = GmO_w.cross(Fg_man);

        v_mom_task = K_O_man + dt * (v_O_world.cross(p_man) + tau_g);

        // Termine opzionale basato sul momento totale h_UAM: -A_KO,b^man * Ab^{-1} * h_UAM
        // (abilitabile anche nel caso pose-mom con redundant=false)
        if (use_h_uam_) {
            pinocchio::computeCentroidalMomentum(model_, data_, q_, v_gen_meas_);
            Eigen::Matrix<double, 6, 1> h_uam;
            h_uam.segment<3>(0) = data_.hg.linear();
            h_uam.segment<3>(3) = data_.hg.angular();
            const Eigen::Matrix<double, 6, 1> Ab_inv_h_uam = Ab_reg.ldlt().solve(h_uam);
            h_uam_term = A_KO_b_man * Ab_inv_h_uam;
            v_mom_task = v_mom_task - h_uam_term;
        }
    }

    // 5. LETTURA INPUT
    geometry_msgs::msg::Pose current_ee_pose_world;
    current_ee_pose_world.position.x = ee_placement.translation().x();
    current_ee_pose_world.position.y = ee_placement.translation().y();
    current_ee_pose_world.position.z = ee_placement.translation().z();
    // Costruisco il quaternion dall'orientazione dell'EE e lo normalizzo
    Eigen::Quaterniond ee_cur_quat(ee_placement.rotation());
    ee_cur_quat.normalize();
    current_ee_pose_world.orientation.x = ee_cur_quat.x();
    current_ee_pose_world.orientation.y = ee_cur_quat.y();
    current_ee_pose_world.orientation.z = ee_cur_quat.z();
    current_ee_pose_world.orientation.w = ee_cur_quat.w();

    // Pubblica la posa assoluta dell'end-effector
    ee_world_pose_pub_->publish(current_ee_pose_world);


    if (desired_ee_velocity_ready_) {
        desired_ee_velocity_vec_ << desired_ee_velocity_world_.linear.x,
                                    desired_ee_velocity_world_.linear.y,
                                    desired_ee_velocity_world_.linear.z,
                                    desired_ee_velocity_world_.angular.x,
                                    desired_ee_velocity_world_.angular.y,
                                    desired_ee_velocity_world_.angular.z;
    }

    // 6. CALCOLO ERRORE POSE END-EFFECTOR

    // - Posizione: errore lineare in world e_p = p_des - p_cur
    const Eigen::Vector3d p_cur = ee_placement.translation();
    const Eigen::Vector3d p_des(desired_ee_pose_world_.position.x,
                                desired_ee_pose_world_.position.y,
                                desired_ee_pose_world_.position.z);
    const Eigen::Vector3d e_pos = p_des - p_cur; // world

    // - Orientazione: errore angolare in world e_w = log( R_des * R_cur^T )
    Eigen::Matrix3d R_cur = ee_placement.rotation();
    Eigen::Quaterniond ee_des_quat(desired_ee_pose_world_.orientation.w,
                                desired_ee_pose_world_.orientation.x,
                                desired_ee_pose_world_.orientation.y,
                                desired_ee_pose_world_.orientation.z);
    ee_des_quat.normalize();
    Eigen::Matrix3d R_des = ee_des_quat.toRotationMatrix();
    Eigen::Matrix3d R_err_world = R_des * R_cur.transpose();
    const Eigen::Vector3d e_ang = pinocchio::log3(R_err_world); // world

    // 7. QP-based control (OSQP-Eigen) - formula identica al prototipo Python
    // Decision variable: x = qdot_arm (size n_arm)
    // Cost:
    //   P = J_task^T * W * J_task + lambda_reg * I  (W = I)
    //   q = - J_task^T * W * v_task
    // Constraints (box): l <= x <= u, con A = I
    //   dq_min <= x <= dq_max
    //   (q_min - q_arm)/dt <= x <= (q_max - q_arm)/dt
    // Intersezione bounds velocità e bounds di posizione (via dt).

    // Stato misurato giunti (ordine arm_joints_) - buffer preallocato
    qp_q_arm_meas_.setZero();
    for (int i = 0; i < n_arm; ++i) {
        const int iq = idx_q_arm_[i];
        if (iq >= 0 && iq < q_.size()) qp_q_arm_meas_[i] = q_[iq];
    }

    // ==== QP cost: due casi ====
    // redundant_=true  (jext): minimize || [Jgen_lin; J_mom] qdot - [v_lin; v_mom] ||
    // redundant_=false (pose-mom): minimize w_kin*||Jgen qdot - v_ee_6d||^2 + w_mom*||J_mom qdot - v_mom||^2

    // v_ee_des = v_ref + Kp*e  (Kp separati pos/orient)
    Eigen::Matrix<double, 6, 1> v_ee_task_6d;
    v_ee_task_6d.segment<3>(0) = desired_ee_velocity_vec_.segment<3>(0) + kp_pos_ * e_pos;
    v_ee_task_6d.segment<3>(3) = desired_ee_velocity_vec_.segment<3>(3) + kp_ori_ * e_ang;

    qp_P_dense_.setZero();
    qp_gradient_.setZero();

    if (redundant_) {
        // Stack Jext (6 x n): [Jgen_lin (3); J_mom (3)]
        qp_J_task_.topRows(3) = Jgen_arm.topRows(3);
        qp_J_task_.bottomRows(3) = J_mom_arm;
        qp_v_task_.head<3>() = v_ee_task_6d.head<3>();
        qp_v_task_.tail<3>() = v_mom_task;

        const auto J_task = qp_J_task_.topRows(6);
        const auto v_task = qp_v_task_.head(6);

        qp_P_dense_.noalias() = J_task.transpose() * J_task;
        qp_P_dense_.diagonal().array() += qp_lambda_reg_;
        qp_P_dense_ = 0.5 * (qp_P_dense_ + qp_P_dense_.transpose());
        qp_gradient_.noalias() = -J_task.transpose() * v_task;
    } else {
        // Due contributi separati e pesati (pose-mom)
        const double w_kin = std::max(0.0, w_kin_);
        const double w_mom = std::max(0.0, w_mom_);

        const Eigen::MatrixXd &J1 = Jgen_arm; // 6 x n
        const Eigen::VectorXd b1 = v_ee_task_6d;
        const Eigen::MatrixXd &J2 = J_mom_arm; // 3 x n
        const Eigen::Vector3d b2 = v_mom_task;

        qp_P_dense_.noalias() = (w_kin * (J1.transpose() * J1)) + (w_mom * (J2.transpose() * J2));
        qp_P_dense_.diagonal().array() += qp_lambda_reg_;
        qp_P_dense_ = 0.5 * (qp_P_dense_ + qp_P_dense_.transpose());

        qp_gradient_.noalias() = -((w_kin * (J1.transpose() * b1)) + (w_mom * (J2.transpose() * b2)));
    }

    // Bounds l/u
    for (int i = 0; i < n_arm; ++i) {
        double vlim = 0.0;
        if (have_velocity_limits_) vlim = v_limit_arm_[i];
        if (!(vlim > 0.0)) vlim = qp_vel_max_default_;
        const double dq_min = -std::abs(vlim);
        const double dq_max =  std::abs(vlim);

        double l_i = dq_min;
        double u_i = dq_max;
        if (have_position_limits_ && std::isfinite(q_lower_arm_[i]) && std::isfinite(q_upper_arm_[i])) {
            const double l_pos = (q_lower_arm_[i] - qp_q_arm_meas_[i]) / dt;
            const double u_pos = (q_upper_arm_[i] - qp_q_arm_meas_[i]) / dt;
            l_i = std::max(l_i, l_pos);
            u_i = std::min(u_i, u_pos);
        }

        qp_l_[i] = l_i;
        qp_u_[i] = u_i;
    }

    // Salvaguardia fattibilità: se l>u forza l=u 
    for (int i = 0; i < n_arm; ++i) {
        if (qp_l_[i] > qp_u_[i]) {
            qp_l_[i] = qp_u_[i];
        }
    }

    // Aggiorna Hessian (upper-triangular) mantenendo pattern fisso
    // Nota: qp_hessian_ contiene solo (i<=j)
    for (int j = 0; j < n_arm; ++j) {
        for (int i = 0; i <= j; ++i) {
            qp_hessian_.coeffRef(i, j) = qp_P_dense_(i, j);
        }
    }

    qp_solution_.setZero();
    bool qp_ok = false;
    if (!qp_initialized_) {
        // Primo setup completo (usa i buffer già allocati)
        qp_solver_.data()->setHessianMatrix(qp_hessian_);
        qp_solver_.data()->setGradient(qp_gradient_);
        qp_solver_.data()->setLowerBound(qp_l_);
        qp_solver_.data()->setUpperBound(qp_u_);
        qp_initialized_ = qp_solver_.initSolver();
        if (!qp_initialized_) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "OSQP-Eigen initSolver() fallito");
        }
    } else {
        qp_solver_.updateHessianMatrix(qp_hessian_);
        qp_solver_.updateGradient(qp_gradient_);
        qp_solver_.updateLowerBound(qp_l_);
        qp_solver_.updateUpperBound(qp_u_);
    }

    if (qp_initialized_) {
        const auto flag = qp_solver_.solveProblem();
        if (flag == OsqpEigen::ErrorExitFlag::NoError) {
            qp_solution_ = qp_solver_.getSolution();
            qp_ok = true;
        } else {
            qp_ok = false;
        }
    }

    if (!qp_ok) {
        // Fallback: x=0 se non ottimo/errore (come richiesto)
        qp_solution_.setZero();
    }

    // Log throttled delle velocità trovate dal QP
    {
        std::ostringstream oss_tot;
        oss_tot.setf(std::ios::fixed);
        oss_tot << std::setprecision(4);
        oss_tot << "qdot_qp [";
        for (int i = 0; i < n_arm; ++i) {
            oss_tot << qp_solution_[i] << (i + 1 < n_arm ? ", " : "");
        }
        oss_tot << "]";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "%s", oss_tot.str().c_str());
    }

    // 8. Integrazione diretta delle velocità QP -> comandi posizione giunti
    const bool need_reinit = (!have_desired_msg_) || ((this->now() - last_desired_msg_time_).seconds() > desired_timeout_sec_);
    if (!arm_cmd_initialized_ || need_reinit) {
        arm_cmd_pos_.assign(arm_joints_.size(), 0.0);
        for (int i = 0; i < n_arm; ++i) {
            arm_cmd_pos_[static_cast<size_t>(i)] = qp_q_arm_meas_[i];
        }
        arm_cmd_initialized_ = true;
    }

    for (int i = 0; i < n_arm; ++i) {
        double q_next = arm_cmd_pos_[static_cast<size_t>(i)] + qp_solution_[i] * dt;
        if (have_position_limits_ && std::isfinite(q_lower_arm_[i]) && std::isfinite(q_upper_arm_[i])) {
            q_next = std::clamp(q_next, q_lower_arm_[i], q_upper_arm_[i]);
        }
        arm_cmd_pos_[static_cast<size_t>(i)] = q_next;
    }

    std_msgs::msg::Float64MultiArray command_msg;
    command_msg.data.resize(arm_joints_.size());
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
        command_msg.data[i] = arm_cmd_pos_[i];
    }

    // Log: angoli di giunto comandati in gradi (ordine: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
    {
        constexpr double RAD2DEG = 57.29577951308232; // 180/pi
        std::ostringstream oss_cmd_deg;
        oss_cmd_deg.setf(std::ios::fixed);
        oss_cmd_deg << std::setprecision(2);
        for (size_t i = 0; i < command_msg.data.size(); ++i) {
            double deg = command_msg.data[i] * RAD2DEG;
            oss_cmd_deg << deg;
            if (i + 1 < command_msg.data.size()) oss_cmd_deg << ", ";
        }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "commanded joint angles [deg] = [%s]", oss_cmd_deg.str().c_str());
    }

    // Pubblicazione del messaggio
    if (real_system_) {
        interbotix_xs_msgs::msg::JointGroupCommand jgc;
        jgc.name = "arm";  // deve corrispondere al gruppo definito nel YAML (groups.arm)
        jgc.cmd.resize(command_msg.data.size());
        for (size_t i = 0; i < command_msg.data.size(); ++i) {
            jgc.cmd[i] = static_cast<float>(command_msg.data[i]);
        }
        if (joint_group_pub_) joint_group_pub_->publish(jgc);
    } else {
        if (arm_controller_pub_) arm_controller_pub_->publish(command_msg);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClikUamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
