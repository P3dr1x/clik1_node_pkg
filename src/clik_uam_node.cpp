#include "clik1_node_pkg/clik_uam_node.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp" // Composite Rigid Body Algorithm (per inerzia)
#include "pinocchio/spatial/se3.hpp" // per SE3 log6
#include "pinocchio/spatial/explog.hpp" // per log3 su SO(3)
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp" // Include necessario per il nuovo tipo di messaggio
#include "px4_ros_com/frame_transforms.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <iomanip>
#include <algorithm>
#include <chrono>


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
    qd_.resize(model_.nv);
    J_.resize(6, model_.nv);
    arm_joints_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};
    declare_parameter("k_err_x_", 50.0); // guadagno posizione traslazionale
    declare_parameter("damping", 1e-4);   // damping per pseudoinversa (Tikhonov)
    // Parametri opzionali per pesi (spalla, forearm_roll, wrist_rotate hanno peso maggiore).
    declare_parameter("shoulder_weight", 15.0);
    declare_parameter("forearm_weight", 15.0);
    declare_parameter("wrist_weight", 15.0);
        // Opzione per sfruttare ridondanza cinematica: segui solo la traiettoria di posizione (ignora orientazione)
        this->declare_parameter<bool>("redundant", false);
        redundant_ = this->get_parameter("redundant").as_bool();
    k_err_x_ = get_parameter("k_err_x_").as_double();
    damping_ = get_parameter("damping").as_double();
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

    // Precalcolo una tantum: indici di velocità e limiti di velocità per i giunti del braccio
    {
        const int n_arm = static_cast<int>(arm_joints_.size());
        idx_v_arm_.resize(n_arm);
        v_max_.resize(n_arm);
        for (int i = 0; i < n_arm; ++i) {
            const std::string &jname = arm_joints_[static_cast<size_t>(i)];
            if (!model_.existJointName(jname)) {
                RCLCPP_ERROR(this->get_logger(), "Joint '%s' non esiste nel modello.", jname.c_str());
                rclcpp::shutdown();
                return;
            }
            const pinocchio::JointIndex jid = model_.getJointId(jname);
            const int idx_v = static_cast<int>(model_.joints[jid].idx_v());
            if (idx_v < 0 || idx_v >= static_cast<int>(model_.nv)) {
                RCLCPP_ERROR(this->get_logger(), "idx_v fuori range per joint '%s' (idx_v=%d, nv=%d)",
                             jname.c_str(), idx_v, static_cast<int>(model_.nv));
                rclcpp::shutdown();
                return;
            }
            idx_v_arm_[i] = idx_v;
            if (idx_v >= 0 && idx_v < model_.velocityLimit.size()) {
                v_max_[i] = model_.velocityLimit[static_cast<Eigen::Index>(idx_v)];
            } else {
                v_max_[i] = 0.0; // 0 -> nessun limite applicato a runtime
            }
        }
        // Log una tantum dei limiti
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(3);
        oss << "v_max [rad/s] precomputati = [";
        for (int i = 0; i < n_arm; ++i) {
            oss << v_max_[i];
            if (i + 1 < n_arm) oss << ", ";
        }
        oss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    // Timer di update() controllo parametrico
    this->declare_parameter<double>("control_rate_hz", 80.0);
    double rate_hz = this->get_parameter("control_rate_hz").as_double();
    rate_hz = std::max(1.0, rate_hz); // salvaguardia: minimo 1 Hz
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    control_timer_ = this->create_wall_timer(period_ns, std::bind(&ClikUamNode::update, this));
    RCLCPP_INFO(this->get_logger(), "control_rate_hz=%.2f Hz (periodo=%.3f ms)", rate_hz, 1000.0 / rate_hz);
    last_update_time_ = this->now();
}

void ClikUamNode::desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    desired_ee_pose_world_ = *msg;
    desired_ee_pose_world_ready_ = true;
    //RCLCPP_INFO(this->get_logger(), "Nuova posa desiderata ricevuta: x=%.3f y=%.3f z=%.3f", msg->position.x, msg->position.y, msg->position.z);
}

void ClikUamNode::desired_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    desired_ee_velocity_world_ = *msg;
    desired_ee_velocity_ready_ = true; // indica che siamo in una fase di tracking
}

void ClikUamNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    // ATTENZIONE: È GIUSTO FLU O ERA MEGLIO NED?
    // Conversione da NED (PX4) a FLU (Forward, Left, Up)
    // NED: x=N (Forward), y=E (Right), z=D (Down)
    // FLU: x=Forward -> x_ned, y=Left -> -y_ned, z=Up -> -z_ned
    vehicle_local_position_ = *msg; // copia originale
    vehicle_local_position_.x = msg->y;        // Forward
    vehicle_local_position_.y = msg->x;       // Left
    vehicle_local_position_.z = -msg->z;       // Up

    has_vehicle_local_position_ = true;
}

void ClikUamNode::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    // Conversione da FRD (PX4) a FLU (ROS2)
    // Eigen::Quaterniond frd_quat(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    // Conversione manuale da FRD (PX4) a FLU (ROS2)
    // FRD (Forward, Right, Down) -> FLU (Forward, Left, Up)
    // La conversione consiste nell'invertire gli assi Y e Z  //VERIFICARE SE È GIUSTO
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

    // --- AGGIORNA STATO DEL ROBOT ---
    // 1. Popola il vettore di configurazione 'q' di Pinocchio
    q_[0] = vehicle_local_position_.x;
    q_[1] = vehicle_local_position_.y;
    q_[2] = vehicle_local_position_.z;
    q_[3] = vehicle_attitude_.q[1]; // x
    q_[4] = vehicle_attitude_.q[2]; // y
    q_[5] = vehicle_attitude_.q[3]; // z
    q_[6] = vehicle_attitude_.q[0]; // w

    // 2. Leggi lo stato attuale dei giunti del braccio usando gli indici corretti di Pinocchio
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
        const auto& joint_name = current_joint_state_.name[i];
        if (!model_.existJointName(joint_name)) continue;
        const pinocchio::JointIndex jid = model_.getJointId(joint_name);
        const int idx_q = static_cast<int>(model_.joints[jid].idx_q());
        const int nq_j = static_cast<int>(model_.joints[jid].nq());
        if (nq_j == 1) {
            if (idx_q >= 0 && idx_q < static_cast<int>(model_.nq)) {
                q_[idx_q] = current_joint_state_.position[i];
            }
        }
    }
    pinocchio::normalize(model_, q_);

    // 3. Velocità (per ora a zero)
    qd_.setZero();

    // --- CALCOLO MATRICI INERZIA E JACOBIANI ---
    // Cinematica diretta per posa assoluta dell'end-effector
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3& ee_placement = data_.oMf[ee_frame_id_];

    // Jacobiano del frame in LOCAL_WORLD_ALIGNED
    // RIC: il vettore velocità desiderata ha convenzione [lin; ang]
    pinocchio::computeFrameJacobian(model_, data_, q_, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_);

    // Sottomatrici Jacobiano: Jb (colonne base) e Jm (colonne braccio)
    Eigen::MatrixXd Jb_mat = J_.leftCols(6);

    // conversione a geometry_msgs::Pose
    geometry_msgs::msg::Pose current_ee_pose_world;
    current_ee_pose_world.position.x = ee_placement.translation().x();
    current_ee_pose_world.position.y = ee_placement.translation().y();
    current_ee_pose_world.position.z = ee_placement.translation().z();
    Eigen::Quaterniond ee_q(ee_placement.rotation());
    current_ee_pose_world.orientation.x = ee_q.x();
    current_ee_pose_world.orientation.y = ee_q.y();
    current_ee_pose_world.orientation.z = ee_q.z();
    current_ee_pose_world.orientation.w = ee_q.w();

    // Pubblica la posa assoluta dell'end-effector
    ee_world_pose_pub_->publish(current_ee_pose_world);
    // --- CONTROLLO: Jm^+ * v_ee_des (6D) ---
    // Estrai Jacobiano del braccio Jm: colonne dei giunti del braccio dopo le 6 della base
    const int n_total = static_cast<int>(model_.nv);
    const int n_arm = static_cast<int>(arm_joints_.size());
    if (n_total < 6 + n_arm) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "nv (%d) < 6 + n_arm (%d): Jacobiano arm non completo", n_total, 6 + n_arm);
        return;
    }

    Eigen::MatrixXd Jm(6, n_arm); Jm.setZero();
    for (int i = 0; i < n_arm; ++i) {
        const int idx_v = idx_v_arm_[i];
        if (idx_v < 0 || idx_v >= static_cast<int>(model_.nv)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "idx_v_arm_ fuori range (idx_v=%d, nv=%d)", idx_v, static_cast<int>(model_.nv));
            return;
        }
        // Colonna intera (6x1) con ordine [lin; ang]
        Jm.col(i) = J_.col(idx_v);
    }

    // Vettore di velocità desiderata dall'esterno (planner): [lin; ang]
    Eigen::Matrix<double, 6, 1> v_ee_des; v_ee_des.setZero();
    if (desired_ee_velocity_ready_) {
        v_ee_des << desired_ee_velocity_world_.linear.x,
                    desired_ee_velocity_world_.linear.y,
                    desired_ee_velocity_world_.linear.z,
                    desired_ee_velocity_world_.angular.x,
                    desired_ee_velocity_world_.angular.y,
                    desired_ee_velocity_world_.angular.z;

        // Log a frequenza limitata (1 Hz) del vettore velocità desiderata [lin; ang]
        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(), *this->get_clock(), 100,
        //     "v_ee_des [lin;ang] = [%.3f %.3f %.3f | %.3f %.3f %.3f]",
        //     v_ee_des(0), v_ee_des(1), v_ee_des(2), v_ee_des(3), v_ee_des(4), v_ee_des(5));
    }

    // Termine di feedback disaccoppiato per LOCAL_WORLD_ALIGNED:
    // - Posizione: errore lineare in world e_p = p_des - p_cur
    // - Orientazione: errore angolare in world e_w = log( R_des * R_cur^T )
    const Eigen::Vector3d p_cur = ee_placement.translation();
    const Eigen::Matrix3d R_cur = ee_placement.rotation();
    Eigen::Quaterniond qd_world(desired_ee_pose_world_.orientation.w,
                                desired_ee_pose_world_.orientation.x,
                                desired_ee_pose_world_.orientation.y,
                                desired_ee_pose_world_.orientation.z);
    qd_world.normalize();
    const Eigen::Matrix3d R_des = qd_world.toRotationMatrix();
    const Eigen::Vector3d p_des(desired_ee_pose_world_.position.x,
                                desired_ee_pose_world_.position.y,
                                desired_ee_pose_world_.position.z);

    const Eigen::Vector3d e_pos = p_des - p_cur; // world
    const Eigen::Matrix3d R_err_world = R_des*R_cur.transpose();
    const Eigen::Vector3d e_ang = pinocchio::log3(R_err_world); // world

    Eigen::Matrix<double,6,1> e6;
    e6.head<3>() = e_pos;
    e6.tail<3>() = e_ang;

    // Separazione feed-forward (q_ik) e feedback (delta_q) in spazio giunti
    // qdot_ik = Jm^+ * v_ff, qdot_fb = Jm^+ * (K * errore)
    Eigen::VectorXd qdot_ik;
    Eigen::VectorXd qdot_fb;
    // Costruisci matrice di peso inverso W^{-1}
    Eigen::VectorXd W_inv_vec = W_diag_.cwiseInverse();
    Eigen::MatrixXd W_inv = W_inv_vec.asDiagonal(); // n_arm x n_arm

    if (redundant_) {
        // usa solo la parte traslazionale: prime 3 righe del Jacobiano e del vettore velocita'
        Eigen::MatrixXd Jm_pos = Jm.topRows(3); // 3 x n_arm
        // A = J * W^{-1} * J^T
        Eigen::MatrixXd A = Jm_pos * W_inv * Jm_pos.transpose(); // 3x3
        Eigen::MatrixXd Areg = A + damping_ * Eigen::MatrixXd::Identity(A.rows(), A.cols());
        Eigen::MatrixXd Ainv = Areg.ldlt().solve(Eigen::MatrixXd::Identity(A.rows(), A.cols()));
        Eigen::MatrixXd Pinv = W_inv * Jm_pos.transpose() * Ainv; // n_arm x 3
        Eigen::Vector3d v_des_pos = v_ee_des.head<3>();
        qdot_ik = Pinv * v_des_pos;                      // n_arm x 1
        qdot_fb = Pinv * (k_err_x_ * e_pos);             // n_arm x 1
    } else {
        // Peso anche nel caso non ridondante
        // A = J * W^{-1} * J^T
        Eigen::MatrixXd A = Jm * W_inv * Jm.transpose(); // 6x6
        Eigen::MatrixXd Areg = A + damping_ * Eigen::MatrixXd::Identity(A.rows(), A.cols());
        Eigen::MatrixXd Ainv = Areg.ldlt().solve(Eigen::MatrixXd::Identity(A.rows(), A.cols()));
        Eigen::MatrixXd Pinv = W_inv * Jm.transpose() * Ainv; // n_arm x 6
        qdot_ik = Pinv * v_ee_des;                         // n_arm x 1
        qdot_fb = Pinv * (k_err_x_ * e6);                  // n_arm x 1
    }

    // Saturazione delle velocità dei giunti in base ai limiti URDF (per-giunto)
    // Applico la saturazione sul contributo totale qdot_tot = qdot_ik + qdot_fb,
    // ridistribuendo proporzionalmente su entrambi i termini per preservarne il rapporto.
    for (int i = 0; i < n_arm; ++i) {
        const double vmax_i = v_max_[i];
        if (vmax_i > 0.0) {
            const double qdot_tot_i = qdot_ik[i] + qdot_fb[i];
            const double abs_tot = std::abs(qdot_tot_i);
            if (abs_tot > vmax_i) {
                const double s = vmax_i / abs_tot; // 0 < s < 1
                qdot_ik[i] *= s;
                qdot_fb[i] *= s;
            }
        }
    }

    // Log throttled delle velocità dei giunti (ik, fb, tot)
    {
        std::ostringstream oss_ik, oss_fb, oss_tot;
        oss_ik.setf(std::ios::fixed); oss_fb.setf(std::ios::fixed); oss_tot.setf(std::ios::fixed);
        oss_ik << std::setprecision(5); oss_fb << std::setprecision(5); oss_tot << std::setprecision(5);
        oss_ik << "qdot_ik [rad/s] = [";
        oss_fb << "qdot_fb [rad/s] = [";
        oss_tot << "qdot_tot [rad/s] = [";
        for (int i = 0; i < qdot_ik.size(); ++i) {
            const double qdot_tot_i = qdot_ik[i] + qdot_fb[i];
            oss_ik << qdot_ik[i] << (i + 1 < qdot_ik.size() ? ", " : "");
            oss_fb << qdot_fb[i] << (i + 1 < qdot_fb.size() ? ", " : "");
            oss_tot << qdot_tot_i << (i + 1 < qdot_ik.size() ? ", " : "");
        }
        oss_ik << "]"; oss_fb << "]"; oss_tot << "]";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "%s | %s | %s",
                             oss_ik.str().c_str(), oss_fb.str().c_str(), oss_tot.str().c_str());
    }

    // Integrazione interna (indipendente dalla frequenza di /joint_states)
    const rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    dt = std::clamp(dt, 0.0, 0.02); // max 20 ms

    // Stato misurato dei giunti (ordinato come arm_joints_)
    Eigen::VectorXd q_meas(n_arm); q_meas.setZero();
    for (int i = 0; i < n_arm; ++i) {
        const std::string &jname = arm_joints_[static_cast<size_t>(i)];
        double q_val = 0.0;
        for (size_t j = 0; j < current_joint_state_.name.size(); ++j) {
            if (current_joint_state_.name[j] == jname) { q_val = current_joint_state_.position[j]; break; }
        }
        q_meas[i] = q_val;
    }

    // Buffer interni: q_ik (feed-forward) e delta_q (feedback)
    static bool ff_fb_initialized = false;
    static Eigen::VectorXd q_ik;    // n_arm
    static Eigen::VectorXd delta_q; // n_arm
    if (!ff_fb_initialized) {
        q_ik = q_meas; // inizializza con misurato
        delta_q = Eigen::VectorXd::Zero(n_arm);
        ff_fb_initialized = true;
    }

    // Integrazione separata
    q_ik    = q_ik    + qdot_ik * dt;      // cinematica inversa prevista
    delta_q = delta_q + qdot_fb * dt;      // correzione di feedback accumulata

    // Configurazione da comandare: q_cmd = q_ik + delta_q
    if (!arm_cmd_initialized_) {
        arm_cmd_pos_.assign(arm_joints_.size(), 0.0);
        arm_cmd_initialized_ = true;
    }
    for (int i = 0; i < n_arm; ++i) {
        arm_cmd_pos_[static_cast<size_t>(i)] = q_ik[i] + delta_q[i];
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
