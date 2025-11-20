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
    q_.setZero();
    qd_.resize(model_.nv);
    qd_.setZero();
    inertia_matrix_.resize(model_.nv, model_.nv);
    J_.resize(6, model_.nv);
    Jgen_.resize(6, model_.nv - 6);
    error_pose_ee_.resize(6);
    desired_ee_velocity_vec_.resize(6);
    desired_ee_velocity_vec_.setZero();
    arm_joints_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"}; // in future they might be specified through yaml file
    declare_parameter("k_err_x_", 20.0); // guadagno posizione traslazionale
    declare_parameter("damping", 1e-4);   // damping per pseudoinversa (Tikhonov)
    // Parametri opzionali per pesi (spalla, forearm_roll, wrist_rotate hanno peso maggiore).
    declare_parameter("shoulder_weight", 15.0);
    declare_parameter("forearm_weight", 25.0);
    declare_parameter("wrist_weight", 25.0);
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
        idx_q_arm_.resize(n_arm);
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
    //qd_.setZero();

    // Cinematica diretta per posa assoluta dell'end-effector
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3& ee_placement = data_.oMf[ee_frame_id_];

    // 3. CALCOLO MATRICI INERZIA E JACOBIANI 

    pinocchio::crba(model_, data_, q_);
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    inertia_matrix_ = data_.M;

    // Jacobiano del frame in LOCAL_WORLD_ALIGNED
    // RIC: il vettore velocità desiderata ha convenzione [lin; ang]
    pinocchio::computeFrameJacobian(model_, data_, q_, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_);

    // Estrai i blocchi necessari
    Eigen::MatrixXd H_b = inertia_matrix_.topLeftCorner(6, 6);
    Eigen::MatrixXd H_bm = inertia_matrix_.topRightCorner(6, model_.nv - 6);
    Eigen::MatrixXd J_b = J_.leftCols(6);
    Eigen::MatrixXd J_m = J_.rightCols(model_.nv - 6);

    // 4. CALCOLO JACOBIANO GENERALIZZATO
    Eigen::MatrixXd Hb_inv_Hbm = H_b.ldlt().solve(H_bm); // risolve H_b * X = H_bm
    Jgen_ = J_m - J_b * Hb_inv_Hbm;

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

    // Converti pose in SE3 di Pinocchio
    pinocchio::SE3 desired_pose_se3(
        pinocchio::SE3::Quaternion(desired_ee_pose_world_.orientation.w, desired_ee_pose_world_.orientation.x, desired_ee_pose_world_.orientation.y, desired_ee_pose_world_.orientation.z),
        Eigen::Vector3d(desired_ee_pose_world_.position.x, desired_ee_pose_world_.position.y, desired_ee_pose_world_.position.z)
    );
    pinocchio::SE3 current_pose_se3(
        pinocchio::SE3::Quaternion(current_ee_pose_world.orientation.w, current_ee_pose_world.orientation.x, current_ee_pose_world.orientation.y, current_ee_pose_world.orientation.z),
        Eigen::Vector3d(current_ee_pose_world.position.x, current_ee_pose_world.position.y, current_ee_pose_world.position.z)
    );

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

    Eigen::Matrix<double,6,1> e6;
    e6.head<3>() = e_pos;
    e6.tail<3>() = e_ang;

    // 7. CALCOLO DELLA VELOCITÀ DEI GIUNTI TRAMITE INVERSIONE PESATA DELLO JACOBIANO GENERALIZZATO
    // Separazione feed-forward (q_ik) e feedback (delta_q) in spazio giunti
    // qdot_ik = Jgen^+ * v_ff, qdot_fb = Jgen^+ * (K * errore)
    Eigen::VectorXd qdot_ik;
    Eigen::VectorXd qdot_fb;
    // Costruisci matrice di peso inverso W^{-1}
    Eigen::VectorXd W_inv_vec = W_diag_.cwiseInverse();
    Eigen::MatrixXd W_inv = W_inv_vec.asDiagonal(); // n_arm x n_arm

    if (redundant_) {
        // Ridondante: usa solo prime 3 righe di Jgen_ (convenzione richiesta) -> Jgen_lin
        Eigen::MatrixXd Jgen_lin = Jgen_arm.topRows(3); // 3 x n_arm
        // A = J * W^{-1} * J^T
        Eigen::MatrixXd A = Jgen_lin * W_inv * Jgen_lin.transpose(); // 3x3
        Eigen::MatrixXd Areg = A + damping_ * Eigen::MatrixXd::Identity(A.rows(), A.cols());
        Eigen::MatrixXd Ainv = Areg.ldlt().solve(Eigen::MatrixXd::Identity(A.rows(), A.cols()));
        Eigen::MatrixXd Pinv = W_inv * Jgen_lin.transpose() * Ainv; // n_arm x 3
        Eigen::Vector3d v_des_pos = desired_ee_velocity_vec_.head<3>();
        qdot_ik = Pinv * v_des_pos;                      // n_arm x 1
        qdot_fb = Pinv * (k_err_x_ * e_pos);             // n_arm x 1
    } else {
        // Caso non ridondante: usa l'intero Jacobiano generalizzato Jgen_ (6 x n_arm)
        // A = Jgen * W^{-1} * Jgen^T
        Eigen::MatrixXd A = Jgen_arm * W_inv * Jgen_arm.transpose(); // 6x6
        Eigen::MatrixXd Areg = A + damping_ * Eigen::MatrixXd::Identity(A.rows(), A.cols());
        Eigen::MatrixXd Ainv = Areg.ldlt().solve(Eigen::MatrixXd::Identity(A.rows(), A.cols()));
        Eigen::MatrixXd Pinv = W_inv * Jgen_arm.transpose() * Ainv; // n_arm x 6
        qdot_ik = Pinv * desired_ee_velocity_vec_;               // n_arm x 1 (feed-forward)
        qdot_fb = Pinv * (k_err_x_ * e6);                        // n_arm x 1 (feedback pose intera)
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

    // Log throttled solo delle velocità totali dei giunti (qdot_tot)
    {
        std::ostringstream oss_tot;
        oss_tot.setf(std::ios::fixed);
        oss_tot << std::setprecision(4);
        oss_tot << "qdot_tot [rad/s] = [";
        for (int i = 0; i < qdot_ik.size(); ++i) {
            const double qdot_tot_i = qdot_ik[i] + qdot_fb[i];
            oss_tot << qdot_tot_i << (i + 1 < qdot_ik.size() ? ", " : "");
        }
        oss_tot << "]";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "%s", oss_tot.str().c_str());
    }

    // 8. INTEGRAZIONE E CALCOLO COMANDO POSIZIONE GIUNTI
    const rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    dt = std::clamp(dt, 0.0, 0.02); // max 20 ms

    // Stato misurato dei giunti del braccio ricavato direttamente da q_ tramite idx_q_arm_
    Eigen::VectorXd q_arm_from_q(n_arm); q_arm_from_q.setZero();
    for (int i = 0; i < n_arm; ++i) {
        int iq = idx_q_arm_[i];
        if (iq >= 0 && iq < q_.size()) q_arm_from_q[i] = q_[iq];
    }

    // Buffer interni: q_ik (feed-forward) e delta_q (feedback)
    static bool ff_fb_initialized = false;
    static Eigen::VectorXd q_ik;    // n_arm
    static Eigen::VectorXd delta_q; // n_arm
    const bool need_reinit = (!have_desired_msg_) || ((this->now() - last_desired_msg_time_).seconds() > desired_timeout_sec_);
    if (!ff_fb_initialized || need_reinit) {
        q_ik = q_arm_from_q; // inizializza o reinizializza con misurato corrente
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
