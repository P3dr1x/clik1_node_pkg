#ifndef CLIK_UAM_NODE_HPP_
#define CLIK_UAM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/fwd.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
// STL headers used in this header (member declarations)
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class ClikUamNode : public rclcpp::Node
{
public:
    ClikUamNode();

private:
    // Metodi
    // get_and_transform_desired_pose rimosso: ora la posa desiderata arriva dal planner esterno
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void real_drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void gazebo_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void gazebo_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void real_drone_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void desired_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update();

    // Variabili membro
    geometry_msgs::msg::Pose desired_ee_pose_world_;
    geometry_msgs::msg::Twist desired_ee_velocity_world_;
    bool desired_ee_pose_world_ready_ = false;
    bool desired_ee_velocity_ready_ = false;
    bool waiting_log_printed_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr real_drone_pose_sub_;
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
    px4_msgs::msg::VehicleAttitude vehicle_attitude_;
    bool has_vehicle_local_position_ = false;
    bool has_vehicle_attitude_ = false;
    bool use_gazebo_pose_;
    bool use_gz_odom_ = true;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gazebo_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr real_drone_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_ee_global_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_ee_velocity_sub_;

    nav_msgs::msg::Odometry gazebo_odom_;
    bool has_gazebo_odom_ = false;
    geometry_msgs::msg::Twist real_drone_twist_;
    bool has_real_drone_twist_ = false;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Pinocchio model and data
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::Data::Matrix6x J_;
    pinocchio::FrameIndex ee_frame_id_;

    // Pinocchio model for the manipulator only (FreeFlyer + arm)
    pinocchio::Model model_man_;
    pinocchio::Data data_man_;
    pinocchio::FrameIndex ee_frame_id_man_;
    pinocchio::FrameIndex arm_base_frame_id_full_;
    std::vector<int> idx_v_arm_man_;
    std::vector<int> idx_q_arm_man_;
    double m_man_tot_ = 0.0;
    Eigen::Vector3d g_world_man_{Eigen::Vector3d::Zero()};

    Eigen::VectorXd q_;
    Eigen::VectorXd qd_;
    Eigen::VectorXd v_gen_meas_;
    Eigen::VectorXd desired_ee_velocity_vec_;
    // LC solution state: weights and previous joint velocities (arm-only)
    Eigen::VectorXd W_diag_;         // diagonal weights for joints (size = n_arm)

    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    // Publisher per Interbotix xs_sdk/xs_sdk_sim
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr joint_group_pub_;
    // Publisher per ros2_control (SITL)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_controller_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_world_pose_pub_;

    sensor_msgs::msg::JointState current_joint_state_;
    bool has_current_joint_state_ = false;

    // Per stimare le velocità giunti quando /joint_states non le fornisce
    rclcpp::Time last_joint_state_time_;
    std::unordered_map<std::string, double> prev_joint_positions_;

    std::vector<std::string> arm_joints_;
    // Indici di velocità per i giunti del braccio (ordine arm_joints_), precomputati una volta
    std::vector<int> idx_v_arm_;
    // Indici di posizione (q) per i giunti del braccio (ordine arm_joints_), precomputati una volta
    std::vector<int> idx_q_arm_;

    // === QP (OSQP-Eigen) state ===
    OsqpEigen::Solver qp_solver_;
    bool qp_initialized_ = false;

    // Dimensioni QP: variabili = giunti controllati (arm_joints_), vincoli = box (A=I)
    int qp_n_ = 0;

    // QP data buffers (pre-allocati)
    Eigen::SparseMatrix<double> qp_hessian_;            // n x n (upper-triangular sparsity)
    Eigen::SparseMatrix<double> qp_A_;                  // n x n (Identity)
    Eigen::VectorXd qp_gradient_;                       // n
    Eigen::VectorXd qp_l_;                              // n
    Eigen::VectorXd qp_u_;                              // n
    Eigen::VectorXd qp_solution_;                       // n

    // Runtime buffers to avoid allocations in update()
    Eigen::VectorXd qp_q_arm_meas_;                     // n

    // Dense work buffers
    Eigen::MatrixXd qp_P_dense_;                        // n x n
    Eigen::MatrixXd qp_J_task_;                         // (3 or 6) x n
    Eigen::VectorXd qp_v_task_;                         // (3 or 6)

    // Limits per joint in arm_joints_ order (if available)
    Eigen::VectorXd q_lower_arm_;
    Eigen::VectorXd q_upper_arm_;
    Eigen::VectorXd v_limit_arm_;
    bool have_position_limits_ = false;
    bool have_velocity_limits_ = false;

    // QP parameters
    double qp_lambda_reg_ = 1e-4;
    double qp_vel_max_default_ = 2.0;
    double k_err_ = 20.0;

    // Weights for pose-mom formulation (redundant=false)
    double w_kin_ = 10.0;
    double w_mom_ = 1.0;

    // Se true, include il termine -A_KO,b^man * Ab^{-1} * h_UAM nel task di momento (solo redundant=true)
    bool use_h_uam_ = false;

    // Parametri runtime
    std::string robot_name_;
    bool real_system_ = true;
    // Se true, ignora l'orientazione e usa solo la parte traslazionale (prime 3 righe del Jacobiano)
    bool redundant_ = false;
    // Integrazione comandi posizione giunti braccio a frequenza di controllo
    std::vector<double> arm_cmd_pos_;
    bool arm_cmd_initialized_ = false;
    rclcpp::Time last_update_time_;

    // Timeout per reinizializzare l'integrazione se mancano riferimenti desiderati
    rclcpp::Time last_desired_msg_time_;
    bool have_desired_msg_ = false;
    double desired_timeout_sec_ = 0.5;
};

#endif // CLIK_UAM_NODE_HPP_
