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
#include "pinocchio/spatial/explog.hpp" // per log3 su SO(3)
#include "ament_index_cpp/get_package_share_directory.hpp"


ClikUamNode::ClikUamNode() : Node("clik_uam_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Nodo clik_uam_node avviato.");

    this->declare_parameter<bool>("use_gazebo_pose", true);
    this->get_parameter("use_gazebo_pose", use_gazebo_pose_);

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
    J_.resize(6, model_.nv);
    Jgen_.resize(6, model_.nv - 6);
    desired_ee_velocity_vec_.resize(6);
    desired_ee_velocity_vec_.setZero();
    // Giunti controllati dal controller (gruppo "arm").
    // Eventuali giunti del gripper vengono comunque letti da /joint_states e aggiornano q_,
    // ma NON sono variabili del QP e NON vengono comandati.
    arm_joints_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"}; // in future they might be specified through yaml file

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
    // Opzione per sfruttare ridondanza cinematica: segui solo la traiettoria di posizione (ignora orientazione)
    this->declare_parameter<bool>("redundant", false);
    redundant_ = this->get_parameter("redundant").as_bool();
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

void ClikUamNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_state_ = *msg;
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

    // 1. AGGIORNA STATO DEL ROBOT 
    // q_.setZero();

    q_[0] = vehicle_local_position_.x;
    q_[1] = vehicle_local_position_.y;
    q_[2] = vehicle_local_position_.z;
    q_[3] = vehicle_attitude_.q[1]; // x
    q_[4] = vehicle_attitude_.q[2]; // y
    q_[5] = vehicle_attitude_.q[3]; // z
    q_[6] = vehicle_attitude_.q[0]; // w

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

    // Velocità generalizzate: assumiamo base ferma (hovering) -> prime 6 componenti zero

    // Cinematica diretta per posa assoluta dell'end-effector
    pinocchio::forwardKinematics(model_, data_, q_);
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

    // Estrai i blocchi del Jacobiano dell'EE
    Eigen::MatrixXd J_b = J_.leftCols(6);
    Eigen::MatrixXd J_m = J_.rightCols(model_.nv - 6);

    // 4. CALCOLO JACOBIANO GENERALIZZATO tramite Ab, Am (Paper_MP Sec. 3.1)
    // Jgen = J_m - J_b * Ab^{-1} * Am
    Eigen::MatrixXd Ab_inv_Am = Ab.ldlt().solve(Am); // risolve Ab * X = Am
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

    const rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    dt = std::clamp(dt, 1e-4, 0.02); // evita divisione per 0, max 20ms

    // Stato misurato giunti (ordine arm_joints_) - buffer preallocato
    qp_q_arm_meas_.setZero();
    for (int i = 0; i < n_arm; ++i) {
        const int iq = idx_q_arm_[i];
        if (iq >= 0 && iq < q_.size()) qp_q_arm_meas_[i] = q_[iq];
    }

    // Task selection (3D position-only se redundant_)
    const int task_dim = redundant_ ? 3 : 6;
    qp_J_task_.topRows(task_dim) = redundant_ ? Jgen_arm.topRows(3) : Jgen_arm;

    // v_ee_des = v_ref + Kp*e  (Kp separati pos/orient)
    qp_v_task_.setZero();
    qp_v_task_.head<3>() = desired_ee_velocity_vec_.head<3>() + kp_pos_ * e_pos;
    qp_v_task_.tail<3>() = desired_ee_velocity_vec_.tail<3>() + kp_ori_ * e_ang;

    // Viste (no alloc) ridotte a task_dim
    const auto J_task = qp_J_task_.topRows(task_dim);
    const auto v_task = qp_v_task_.head(task_dim);

    // P_dense = J^T J + lambda I (W = I)
    qp_P_dense_.noalias() = J_task.transpose() * J_task;
    qp_P_dense_.diagonal().array() += qp_lambda_reg_;
    // Symmetrize numerically
    qp_P_dense_ = 0.5 * (qp_P_dense_ + qp_P_dense_.transpose());

    // gradient = -J^T v
    qp_gradient_.noalias() = -J_task.transpose() * v_task;

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

    // Salvaguardia fattibilità: se l>u forza l=u (come nello script Python)
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
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "%s", oss_tot.str().c_str());
    }

    // 8. Integrazione diretta delle velocità QP -> comandi posizione giunti
    // (senza separare IK/feedback, come richiesto)
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
