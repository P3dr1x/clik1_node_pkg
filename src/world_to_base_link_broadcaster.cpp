#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>

class WorldToBaseLinkBroadcaster : public rclcpp::Node {
public:
    WorldToBaseLinkBroadcaster() : Node("world_to_base_link_broadcaster") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        this->declare_parameter<bool>("real_system", false);
        this->get_parameter("real_system", real_system_);

        if (real_system_) {
            RCLCPP_INFO(this->get_logger(), "Utilizzo della posa reale da Motion Capture (/t960a/pose - PoseStamped).");
            real_drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/t960a/pose", 10,
                std::bind(&WorldToBaseLinkBroadcaster::real_drone_pose_callback, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "Utilizzo della posa da Gazebo (/world/default/dynamic_pose/info)." );
            drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/world/default/dynamic_pose/info", 10,
                std::bind(&WorldToBaseLinkBroadcaster::gazebo_pose_callback, this, std::placeholders::_1));
        }

        transform_timer_ = rclcpp::create_timer(
            this->get_node_base_interface(),
            this->get_node_timers_interface(),
            this->get_clock(),
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&WorldToBaseLinkBroadcaster::broadcast_world_to_base_link_tf, this));
    }

private:
    void broadcast_world_to_base_link_tf() {
        if (!has_vehicle_local_position_ || !has_vehicle_attitude_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Dati di posizione o orientamento del drone non disponibili.");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = vehicle_local_position_.position.x;
        transform.transform.translation.y = vehicle_local_position_.position.y;
        transform.transform.translation.z = vehicle_local_position_.position.z;

        transform.transform.rotation.x = vehicle_attitude_.orientation.x;
        transform.transform.rotation.y = vehicle_attitude_.orientation.y;
        transform.transform.rotation.z = vehicle_attitude_.orientation.z;
        transform.transform.rotation.w = vehicle_attitude_.orientation.w;

        tf_broadcaster_->sendTransform(transform);
    }

    void gazebo_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!msg->poses.empty()) {
            const auto& pose = msg->poses[0];
            vehicle_local_position_.position.x = pose.position.x;
            vehicle_local_position_.position.y = pose.position.y;
            vehicle_local_position_.position.z = pose.position.z;

            vehicle_attitude_.orientation.w = pose.orientation.w;
            vehicle_attitude_.orientation.x = pose.orientation.x;
            vehicle_attitude_.orientation.y = pose.orientation.y;
            vehicle_attitude_.orientation.z = pose.orientation.z;

            has_vehicle_local_position_ = true;
            has_vehicle_attitude_ = true;
        }
    }

    // void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {}
    // void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {}

    void real_drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        vehicle_local_position_.position = msg->pose.position;
        vehicle_attitude_.orientation = msg->pose.orientation;
        has_vehicle_local_position_ = true;
        has_vehicle_attitude_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr real_drone_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr transform_timer_;

    bool real_system_;
    bool has_vehicle_local_position_ = false;
    bool has_vehicle_attitude_ = false;
    geometry_msgs::msg::Pose vehicle_local_position_;
    geometry_msgs::msg::Pose vehicle_attitude_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldToBaseLinkBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
