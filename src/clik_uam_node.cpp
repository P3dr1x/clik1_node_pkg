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
        RCLCPP_INFO(this->get_logger(), "Utilizzo della posa dal nodo real_drone_pose_pub (/real_t960a_pose).");
        real_drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/real_t960a_pose", 10, std::bind(&ClikUamNode::real_drone_pose_callback, this, std::placeholders::_1));
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
    inertia_matrix_.resize(model_.nv, model_.nv);
    J_.resize(6, model_.nv);
    Jgen_.resize(6, model_.nv - 6);
    error_pose_ee_.resize(6);
    desired_ee_velocity_vec_.setZero(6);
    arm_joints_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};

    declare_parameter("k_err_x_", 50.0);
    k_err_x_ = get_parameter("k_err_x_").as_double();
    K_matrix_ = Eigen::MatrixXd::Identity(6, 6) * k_err_x_;

    // Timer controllo
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ClikUamNode::update, this));  // 100 Hz
}

void ClikUamNode::desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    desired_ee_pose_world_ = *msg;
    desired_ee_pose_world_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Nuova posa desiderata ricevuta: x=%.3f y=%.3f z=%.3f", msg->position.x, msg->position.y, msg->position.z);
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

void ClikUamNode::real_drone_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Assumiamo che /real_t960a_pose sia già in frame world FLU con yaw eliminato (o come concordato)
    vehicle_local_position_.x = msg->position.x;
    vehicle_local_position_.y = msg->position.y;
    vehicle_local_position_.z = msg->position.z;

    vehicle_attitude_.q[0] = msg->orientation.w;
    vehicle_attitude_.q[1] = msg->orientation.x;
    vehicle_attitude_.q[2] = msg->orientation.y;
    vehicle_attitude_.q[3] = msg->orientation.z;
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

    // --- CALCOLO RIFERIMENTI DI VELOCITÀ ---
    // desired_ee_velocity è presa dal topic /desired_ee_velocity; se non in tracking, resta nulla
    if (desired_ee_velocity_ready_) {
        desired_ee_velocity_vec_.resize(6);
        desired_ee_velocity_vec_.setZero();
        desired_ee_velocity_vec_(0) = desired_ee_velocity_world_.linear.x;
        desired_ee_velocity_vec_(1) = desired_ee_velocity_world_.linear.y;
        desired_ee_velocity_vec_(2) = desired_ee_velocity_world_.linear.z;
        desired_ee_velocity_vec_(3) = desired_ee_velocity_world_.angular.x;
        desired_ee_velocity_vec_(4) = desired_ee_velocity_world_.angular.y;
        desired_ee_velocity_vec_(5) = desired_ee_velocity_world_.angular.z;
    } else {
        if (desired_ee_velocity_vec_.size() != 6) desired_ee_velocity_vec_.resize(6);
        desired_ee_velocity_vec_.setZero();
    }
    Eigen::VectorXd desired_joint_velocities = Jgen_.completeOrthogonalDecomposition().pseudoInverse() * (desired_ee_velocity_vec_ + K_matrix_ * error_pose_ee_);

    // Integrazione per ottenere posizione
    double dt = 0.01; // 100Hz
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
