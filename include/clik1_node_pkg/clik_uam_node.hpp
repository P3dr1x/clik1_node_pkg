#ifndef CLIK_UAM_NODE_HPP_
#define CLIK_UAM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

class ClikUamNode : public rclcpp::Node
{
public:
    ClikUamNode();

private:
    // Metodi
    void get_desired_pose_from_user();
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void gazebo_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void transform_pose();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void update();

    // Metodi per pubblicazre ciò che serve alla visualizzazione in Rviz
    void publish_desired_global_pose(const geometry_msgs::msg::Pose& pose);

    // Variabili membro
    geometry_msgs::msg::Pose desired_ee_pose_local_;
    geometry_msgs::msg::Pose desired_ee_pose_world_;
    bool desired_ee_pose_world_ready_ = false;

    // Subscribers ai topics dove sono pubblicati i dati di posa del drone
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
    px4_msgs::msg::VehicleAttitude vehicle_attitude_;
    bool has_vehicle_local_position_ = false;
    bool has_vehicle_attitude_ = false;
    bool use_gazebo_pose_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gazebo_pose_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Pose last_published_pose_;

    // Pinocchio model and data
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::Data::Matrix6x J_;
    pinocchio::FrameIndex ee_frame_id_;

    Eigen::MatrixXd Jgen_;
    Eigen::MatrixXd inertia_matrix_;
    Eigen::VectorXd q_;
    Eigen::VectorXd qd_;
    Eigen::VectorXd error_pose_ee_;
    Eigen::MatrixXd K_matrix_;

    rclcpp::TimerBase::SharedPtr transform_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_world_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_ee_global_pose_pub_;

    sensor_msgs::msg::JointState current_joint_state_;
    bool has_current_joint_state_ = false;

    double k_err_x_; // Guadagno proporzionale configurabile

    std::vector<std::string> arm_joints_;
};

#endif // CLIK_UAM_NODE_HPP_
