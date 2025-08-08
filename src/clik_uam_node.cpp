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
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp" // Include necessario per il nuovo tipo di messaggio
#include "px4_ros_com/frame_transforms.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

ClikUamNode::ClikUamNode() : Node("clik_uam_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Nodo clik_uam_node avviato.");

    this->declare_parameter<bool>("use_gazebo_pose", true);
    this->get_parameter("use_gazebo_pose", use_gazebo_pose_);

    // Carica il modello URDF
    // NOTA: il percorso del file URDF potrebbe dover essere reso un parametro
    const auto pkg_share = ament_index_cpp::get_package_share_directory("clik1_node_pkg");
    const std::string urdf_filename = pkg_share + "/model/t960a.urdf";
    
    RCLCPP_INFO(this->get_logger(), "Caricamento modello URDF da: %s", urdf_filename.c_str());
    
    try {
        pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model_);
        data_ = pinocchio::Data(model_);
        
        RCLCPP_INFO(this->get_logger(), "Modello URDF caricato con successo.");
        // RCLCPP_INFO(this->get_logger(), "Numero di frame: %lu", model_.frames.size());
        // RCLCPP_INFO(this->get_logger(), "Numero di joint: %d", model_.njoints);
        // RCLCPP_INFO(this->get_logger(), "Dimensioni q: %d", model_.nq);
        
        // Debug: stampa alcuni frame importanti
        for (size_t i = 0; i < model_.frames.size(); ++i) {
            const auto& frame_name = model_.frames[i].name;
            if (frame_name.find("mobile_wx250s") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Frame trovato: %s", frame_name.c_str());
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento del modello URDF: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // Controlla se i frame necessari esistono
    if (!model_.existFrame("mobile_wx250s/ee_gripper_link"))
    {
        RCLCPP_ERROR(this->get_logger(), "Frame 'mobile_wx250s/ee_gripper_link' does not exist in the model. Shutting down.");
        rclcpp::shutdown();
        return;
    }
    ee_frame_id_ = model_.getFrameId("mobile_wx250s/ee_gripper_link");
    RCLCPP_INFO(this->get_logger(), "End-effector frame ID: %d", static_cast<int>(ee_frame_id_));

    // Inizializza il broadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // CREAZIONE TOPIC E SOTTOSCRIZIONI
    if (use_gazebo_pose_) {
        RCLCPP_INFO(this->get_logger(), "Utilizzo della posa da Gazebo (/world/default/dynamic_pose/info).");
        gazebo_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/world/default/dynamic_pose/info", 10, std::bind(&ClikUamNode::gazebo_pose_callback, this, std::placeholders::_1));
    } else {
        RCLCPP_INFO(this->get_logger(), "Utilizzo della posa da PX4 (/fmu/out/...).");
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(), std::bind(&ClikUamNode::vehicle_local_position_callback, this, std::placeholders::_1));
        vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(), std::bind(&ClikUamNode::vehicle_attitude_callback, this, std::placeholders::_1));
    }
        
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&ClikUamNode::joint_state_callback, this, std::placeholders::_1));

    // Modifica: Cambiato il tipo di publisher e il topic
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands", 10);
    
    // Publisher per la posa dell'end-effector nel frame 'world'
    ee_world_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ee_world_pose", 10);

    // Aggiungo un publisher per la posa desiderata globale dell'end-effector
    desired_ee_global_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/desired_ee_global_pose", rclcpp::QoS(10).transient_local());

    // RICHIESTA DELLA POSA DELL'END-EFFECTOR DESIDERATA ALL'UTENTE (posa rispetto alla base del braccio 'mobile_wx250s/base_link')
    get_and_transform_desired_pose();

    // Ridimensiona vettori e matrici
    q_.resize(model_.nq);
    qd_.resize(model_.nv);
    inertia_matrix_.resize(model_.nv, model_.nv);
    J_.resize(6, model_.nv);
    Jgen_.resize(6, model_.nv - 6);
    error_pose_ee_.resize(6);
    // Definisco vettore contenente i nomi dei giunti del braccio
    arm_joints_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};
    // Dichiarazione del parametro per il guadagno proporzionale
    declare_parameter("k_err_x_", 10.0);
    k_err_x_ = get_parameter("k_err_x_").as_double();

    // Inizializzazione di K_matrix_ con il valore configurato
    K_matrix_ = Eigen::MatrixXd::Identity(6, 6) * k_err_x_;

    // // Timer per eseguire la trasformazione quando i dati sono disponibili
    // transform_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100), std::bind(&ClikUamNode::transform_pose, this)); // 10 Hz

    // CREAZIONE TOPIC E SOTTOSCRIZIONI
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&ClikUamNode::joint_state_callback, this, std::placeholders::_1));

    // Modifica: Cambiato il tipo di publisher e il topic
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands", 10);
    
    // Publisher per la posa dell'end-effector nel frame 'world'
    ee_world_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ee_world_pose", 10);

    // Aggiungo un publisher per la posa desiderata globale dell'end-effector
    desired_ee_global_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/desired_ee_global_pose", rclcpp::QoS(10).transient_local());

    // Timer per il ciclo di controllo principale (update)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ClikUamNode::update, this)); // 100 Hz

}

void ClikUamNode::get_and_transform_desired_pose()
{
    geometry_msgs::msg::Pose desired_pose_local;
    std::string input;

    RCLCPP_INFO(this->get_logger(), "Inserire la posa desiderata per l'end-effector (rispetto a 'mobile_wx250s/base_link').");
    RCLCPP_INFO(this->get_logger(), "Formato: 'x y z qx qy qz qw' (7 valori separati da spazio).");
    RCLCPP_INFO(this->get_logger(), "Premere INVIO per usare la posa di default [0.45 0 0.36 0 0 0 1].");
    std::cout << "> ";
    std::getline(std::cin, input);

    std::stringstream ss(input);
    double val;
    std::vector<double> values;
    while (ss >> val)
    {
        values.push_back(val);
    }

    if (values.size() == 7)
    {
        desired_pose_local.position.x = values[0];
        desired_pose_local.position.y = values[1];
        desired_pose_local.position.z = values[2];
        desired_pose_local.orientation.x = values[3];
        desired_pose_local.orientation.y = values[4];
        desired_pose_local.orientation.z = values[5];
        desired_pose_local.orientation.w = values[6];
        RCLCPP_INFO(this->get_logger(), "Posa desiderata impostata dall'utente.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Input non valido o assente. Calcolo della posa di default (home) con Pinocchio.");

        // Usa una copia temporanea di Data per non "sporcare" l'oggetto membro data_
        pinocchio::Data data_home(model_);

        // Calcola la posa di home usando Pinocchio
        // 1. Ottieni la configurazione neutrale (tutti i giunti a zero)
        const Eigen::VectorXd q_home = pinocchio::neutral(model_);

        // 2. Esegui la cinematica diretta per questa configurazione
        // pinocchio::forwardKinematics(model_, data_home, q_home);
        // pinocchio::updateFramePlacements(model_, data_home);
        pinocchio::framesForwardKinematics(model_, data_home, q_home);

        // 3. Ottieni l'ID del frame della base del braccio
        const pinocchio::FrameIndex arm_base_frame_id = model_.getFrameId("mobile_wx250s/base_link");

        // 4. Estrai le pose dei frame di interesse rispetto al frame 'world'
        const pinocchio::SE3& T_world_ee = data_home.oMf[ee_frame_id_];
        const pinocchio::SE3& T_world_arm_base = data_home.oMf[arm_base_frame_id];

        // 5. Calcola la posa relativa dell'end-effector rispetto alla base del braccio
        const pinocchio::SE3 T_arm_base_ee = T_world_arm_base.inverse() * T_world_ee;

        // 6. Converti la SE3 in geometry_msgs::Pose
        desired_pose_local.position.x = T_arm_base_ee.translation().x();
        desired_pose_local.position.y = T_arm_base_ee.translation().y();
        desired_pose_local.position.z = T_arm_base_ee.translation().z();

        Eigen::Quaterniond q_local(T_arm_base_ee.rotation());
        desired_pose_local.orientation.x = q_local.x();
        desired_pose_local.orientation.y = q_local.y();
        desired_pose_local.orientation.z = q_local.z();
        desired_pose_local.orientation.w = q_local.w();
    }

    this->desired_ee_pose_local_ = desired_pose_local;

    RCLCPP_INFO(this->get_logger(), "Posa desiderata (locale): x=%.2f, y=%.2f, z=%.2f, qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f",
                desired_ee_pose_local_.position.x, desired_ee_pose_local_.position.y, desired_ee_pose_local_.position.z,
                desired_ee_pose_local_.orientation.x, desired_ee_pose_local_.orientation.y, desired_ee_pose_local_.orientation.z, desired_ee_pose_local_.orientation.w);

    // Attendi che i dati di posa del drone siano disponibili
    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok() && (!has_vehicle_local_position_ || !has_vehicle_attitude_))
    {
        RCLCPP_WARN(this->get_logger(), "In attesa dei dati di posizione e assetto del veicolo...");
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
    
    if (!has_vehicle_local_position_ || !has_vehicle_attitude_)
    {
        RCLCPP_ERROR(this->get_logger(), "Posizione o assetto del veicolo non disponibili. Impossibile calcolare la posa globale dell'EE.");
        return;
    }

    // Aggiorna la cinematica di Pinocchio una volta per ottenere le trasformazioni iniziali
    pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
    pinocchio::updateFramePlacements(model_, data_);

    // Ottieni la trasformazione statica da base_link a mobile_wx250s/base_link dall'URDF
    const pinocchio::FrameIndex frame_id = model_.getFrameId("mobile_wx250s/base_link");
    const pinocchio::SE3& tf_base_to_arm_base = data_.oMf[frame_id];

    // Crea la posa del drone (base_link) nel frame 'world'
    geometry_msgs::msg::Pose drone_pose;
    drone_pose.position.x = vehicle_local_position_.x;
    drone_pose.position.y = vehicle_local_position_.y;
    drone_pose.position.z = vehicle_local_position_.z;
    drone_pose.orientation.x = vehicle_attitude_.q[1];
    drone_pose.orientation.y = vehicle_attitude_.q[2];
    drone_pose.orientation.z = vehicle_attitude_.q[3];
    drone_pose.orientation.w = vehicle_attitude_.q[0];

    // In Pinocchio, il quaternione è (x, y, z, w)
    // In ROS, il quaternione è (x, y, z, w)
    // In PX4, il quaternione è (w, x, y, z)
    // La conversione gestisce già questo, ma è bene ricordarlo.

    // Trasforma la posa desiderata da locale (rispetto a mobile_wx250s/base_link) a 'world'
    tf2::Transform tf_drone_pose;
    tf2::fromMsg(drone_pose, tf_drone_pose);

    tf2::Transform tf_arm_base_to_local_pose;
    tf2::fromMsg(desired_ee_pose_local_, tf_arm_base_to_local_pose);

    tf2::Transform tf_base_to_arm_base_tf2;
    tf_base_to_arm_base_tf2.setOrigin(tf2::Vector3(tf_base_to_arm_base.translation().x(), tf_base_to_arm_base.translation().y(), tf_base_to_arm_base.translation().z()));
    // Estrai la rotazione come quaternion da Eigen::Matrix3d
    Eigen::Quaterniond eigen_quat(tf_base_to_arm_base.rotation());
    tf2::Quaternion tf2_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    tf_base_to_arm_base_tf2.setRotation(tf2_quat);

    tf2::Transform tf_world_to_desired_pose = tf_drone_pose * tf_base_to_arm_base_tf2 * tf_arm_base_to_local_pose;

    tf2::toMsg(tf_world_to_desired_pose, desired_ee_pose_world_);

    RCLCPP_INFO(this->get_logger(), "Posa desiderata (world): x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                desired_ee_pose_world_.position.x, desired_ee_pose_world_.position.y, desired_ee_pose_world_.position.z,
                desired_ee_pose_world_.orientation.x, desired_ee_pose_world_.orientation.y, desired_ee_pose_world_.orientation.z, desired_ee_pose_world_.orientation.w);

    publish_desired_global_pose(desired_ee_pose_world_);
    desired_ee_pose_world_ready_ = true;
}

void ClikUamNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    // Conversione da NED (PX4) a ENU (ROS2)
    Eigen::Vector3d ned_pos(msg->x, msg->y, msg->z);
    Eigen::Vector3d enu_pos = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_pos);

    vehicle_local_position_ = *msg; // Copio il messaggio originale
    // Sovrascrivo con le coordinate convertite
    vehicle_local_position_.x = enu_pos.x();
    vehicle_local_position_.y = enu_pos.y();
    vehicle_local_position_.z = enu_pos.z();

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

// void ClikUamNode::transform_pose()
// {
//     if (!has_vehicle_local_position_ || !has_vehicle_attitude_) {
//         return;
//     }

//     // Aggiorna la cinematica di Pinocchio una volta per ottenere le trasformazioni iniziali
//     pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
//     pinocchio::updateFramePlacements(model_, data_);

//     // Ottieni la trasformazione statica da base_link a mobile_wx250s/base_link dall'URDF
//     const pinocchio::FrameIndex frame_id = model_.getFrameId("mobile_wx250s/base_link");
//     const pinocchio::SE3& tf_base_to_arm_base = data_.oMf[frame_id];

//     // Crea la posa del drone (base_link) nel frame 'world'
//     geometry_msgs::msg::Pose drone_pose;
//     drone_pose.position.x = vehicle_local_position_.x;
//     drone_pose.position.y = vehicle_local_position_.y;
//     drone_pose.position.z = vehicle_local_position_.z;
//     drone_pose.orientation.x = vehicle_attitude_.q[1];
//     drone_pose.orientation.y = vehicle_attitude_.q[2];
//     drone_pose.orientation.z = vehicle_attitude_.q[3];
//     drone_pose.orientation.w = vehicle_attitude_.q[0];

//     // In Pinocchio, il quaternione è (x, y, z, w)
//     // In ROS, il quaternione è (x, y, z, w)
//     // In PX4, il quaternione è (w, x, y, z)
//     // La conversione gestisce già questo, ma è bene ricordarlo.

//     // Trasforma la posa desiderata da locale (rispetto a mobile_wx250s/base_link) a 'world'
//     tf2::Transform tf_drone_pose;
//     tf2::fromMsg(drone_pose, tf_drone_pose);

//     tf2::Transform tf_arm_base_to_local_pose;
//     tf2::fromMsg(desired_ee_pose_local_, tf_arm_base_to_local_pose);

//     tf2::Transform tf_base_to_arm_base_tf2;
//     tf_base_to_arm_base_tf2.setOrigin(tf2::Vector3(tf_base_to_arm_base.translation().x(), tf_base_to_arm_base.translation().y(), tf_base_to_arm_base.translation().z()));
//     // Estrai la rotazione come quaternion da Eigen::Matrix3d
//     Eigen::Quaterniond eigen_quat(tf_base_to_arm_base.rotation());
//     tf2::Quaternion tf2_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
//     tf_base_to_arm_base_tf2.setRotation(tf2_quat);

//     tf2::Transform tf_world_to_desired_pose = tf_drone_pose * tf_base_to_arm_base_tf2 * tf_arm_base_to_local_pose;

//     tf2::toMsg(tf_world_to_desired_pose, desired_ee_pose_world_);

//     RCLCPP_INFO_ONCE(this->get_logger(), "Posa desiderata (world) calcolata.");
//     RCLCPP_INFO(this->get_logger(), "Posa desiderata (world): x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
//         desired_ee_pose_world_.position.x,
//         desired_ee_pose_world_.position.y,
//         desired_ee_pose_world_.position.z,
//         desired_ee_pose_world_.orientation.x,
//         desired_ee_pose_world_.orientation.y,
//         desired_ee_pose_world_.orientation.z,
//         desired_ee_pose_world_.orientation.w);

//     // Pubblica la posa desiderata una sola volta
//     publish_desired_global_pose(desired_ee_pose_world_);

//     // Disattiva il timer dopo la prima esecuzione
//     transform_timer_->cancel();
//     desired_ee_pose_world_ready_ = true;
// }

void ClikUamNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_state_ = *msg;
    has_current_joint_state_ = true;
}

void ClikUamNode::update()
{
    if (!desired_ee_pose_world_ready_ || !has_current_joint_state_ || current_joint_state_.name.empty() || !has_vehicle_local_position_ || !has_vehicle_attitude_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Dati non pronti per l'update.");
        return;
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

    // 2. Leggi lo stato attuale dei giunti del braccio
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

    // 3. Velocità (per ora a zero)
    qd_.setZero();

    // --- CALCOLO MATRICI INERZIA E JACOBIANI ---
    pinocchio::crba(model_, data_, q_);
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    inertia_matrix_ = data_.M;

    pinocchio::computeFrameJacobian(model_, data_, q_, ee_frame_id_, pinocchio::ReferenceFrame::WORLD, J_);

    // cinematica diretta per posa assoluta dell'end-effector
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3& ee_placement = data_.oMf[ee_frame_id_];
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

    // Estrai i blocchi necessari
    Eigen::MatrixXd H_b = inertia_matrix_.topLeftCorner(6, 6);
    Eigen::MatrixXd H_bm = inertia_matrix_.topRightCorner(6, model_.nv - 6);
    Eigen::MatrixXd J_b = J_.leftCols(6);
    Eigen::MatrixXd J_m = J_.rightCols(model_.nv - 6);

    // --- CALCOLO JACOBIANO GENERALIZZATO ---
    Jgen_ = J_m - J_b * H_b.inverse() * H_bm;

    // --- CALCOLO ERRORE DI POSA ---
    // Converti pose in SE3 di Pinocchio
    pinocchio::SE3 desired_pose_se3(
        pinocchio::SE3::Quaternion(desired_ee_pose_world_.orientation.w, desired_ee_pose_world_.orientation.x, desired_ee_pose_world_.orientation.y, desired_ee_pose_world_.orientation.z),
        Eigen::Vector3d(desired_ee_pose_world_.position.x, desired_ee_pose_world_.position.y, desired_ee_pose_world_.position.z)
    );
    pinocchio::SE3 current_pose_se3(
        pinocchio::SE3::Quaternion(current_ee_pose_world.orientation.w, current_ee_pose_world.orientation.x, current_ee_pose_world.orientation.y, current_ee_pose_world.orientation.z),
        Eigen::Vector3d(current_ee_pose_world.position.x, current_ee_pose_world.position.y, current_ee_pose_world.position.z)
    );

    // Calcola l'errore 6D
    error_pose_ee_ = pinocchio::log6(desired_pose_se3 * current_pose_se3.inverse()).toVector();

    // --- CALCOLO RIFERIMENTI DI POSIZIONE ---
    Eigen::VectorXd desired_ee_velocity = K_matrix_ * error_pose_ee_;
    Eigen::VectorXd desired_joint_velocities = Jgen_.completeOrthogonalDecomposition().pseudoInverse() * desired_ee_velocity;

    // Integrazione per ottenere posizione
    double dt = 0.01; // 100Hz

    // Modifica: Creazione del messaggio Float64MultiArray
    std_msgs::msg::Float64MultiArray command_msg;
    command_msg.data.resize(arm_joints_.size());
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
        double current_pos = 0.0;
        const auto& joint_name_to_find = arm_joints_[i];
        for (size_t j = 0; j < current_joint_state_.name.size(); ++j) {
            if (current_joint_state_.name[j] == joint_name_to_find) {
                current_pos = current_joint_state_.position[j];
                break;
            }
        }
        command_msg.data[i] = current_pos + desired_joint_velocities(i) * dt;
    }

    // Pubblicazione del messaggio
    joint_command_pub_->publish(command_msg);

}

void ClikUamNode::publish_desired_global_pose(const geometry_msgs::msg::Pose& pose) {
    if (pose != last_published_pose_) {
        desired_ee_global_pose_pub_->publish(pose);
        last_published_pose_ = pose;
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
