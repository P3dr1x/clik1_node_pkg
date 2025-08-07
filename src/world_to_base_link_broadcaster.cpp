#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>

class WorldToBaseLinkBroadcaster : public rclcpp::Node {
public:
    WorldToBaseLinkBroadcaster() : Node("world_to_base_link_broadcaster") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        this->declare_parameter<bool>("use_gazebo_pose", true);
        this->get_parameter("use_gazebo_pose", use_gazebo_pose_);

        if (use_gazebo_pose_) {
            RCLCPP_INFO(this->get_logger(), "Utilizzo della posa da Gazebo (/world/default/dynamic_pose/info)." );
            drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/world/default/dynamic_pose/info", 10,
                std::bind(&WorldToBaseLinkBroadcaster::gazebo_pose_callback, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "Utilizzo della posa da PX4 (/fmu/out/...).");
            vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
                std::bind(&WorldToBaseLinkBroadcaster::vehicle_local_position_callback, this, std::placeholders::_1));
            vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
                "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
                std::bind(&WorldToBaseLinkBroadcaster::vehicle_attitude_callback, this, std::placeholders::_1));
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

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        Eigen::Vector3d ned_pos(msg->x, msg->y, msg->z);
        Eigen::Vector3d enu_pos(ned_pos.y(), ned_pos.x(), -ned_pos.z());

        vehicle_local_position_.position.x = enu_pos.x();
        vehicle_local_position_.position.y = enu_pos.y();
        vehicle_local_position_.position.z = enu_pos.z();

        has_vehicle_local_position_ = true;
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        Eigen::Quaterniond flu_quat(msg->q[0], msg->q[1], -msg->q[2], -msg->q[3]);

        vehicle_attitude_.orientation.w = flu_quat.w();
        vehicle_attitude_.orientation.x = flu_quat.x();
        vehicle_attitude_.orientation.y = flu_quat.y();
        vehicle_attitude_.orientation.z = flu_quat.z();

        has_vehicle_attitude_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr transform_timer_;

    bool use_gazebo_pose_;
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
