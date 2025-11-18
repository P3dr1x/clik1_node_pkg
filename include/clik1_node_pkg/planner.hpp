#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/fwd.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "px4_ros_com/frame_transforms.h"
#include <thread>
#include <sstream>
#include <vector>
#include <iostream>

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();
  void run();

private:
  void get_and_transform_desired_pose(); // Copiata identica da clik_uam_node.cpp
  void run_circular_trajectory();
  void run_polyline_trajectory();
  void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void real_drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gazebo_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void publish_desired_global_pose(const geometry_msgs::msg::Pose& pose);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Variabili membro necessarie (stesse della funzione copiata)
  geometry_msgs::msg::Pose desired_ee_pose_local_;
  geometry_msgs::msg::Pose desired_ee_pose_world_;
  bool desired_ee_pose_world_ready_ = false;
  geometry_msgs::msg::Pose last_published_pose_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_ee_global_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desired_ee_velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState current_joint_state_;
  bool has_joint_state_ = false;
  // Subscription opzionale alla posa reale dell'EE pubblicata dal controller (se presente)
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ee_world_pose_sub_;
  geometry_msgs::msg::Pose current_ee_pose_;
  bool has_current_ee_pose_ = false;

  // Subscribers pose drone
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gazebo_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr real_drone_pose_sub_;
  px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
  px4_msgs::msg::VehicleAttitude vehicle_attitude_;
  bool has_vehicle_local_position_ = false;
  bool has_vehicle_attitude_ = false;
  bool use_gazebo_pose_;

  // TF (anche se non strettamente usati qui, mantengo per identicità logica)
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Pinocchio
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_;
};

#endif // PLANNER_HPP_
