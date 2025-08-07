#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>

class RvizMarkerNode : public rclcpp::Node {
public:
    RvizMarkerNode() : Node("rviz_marker_node") {

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscriber per la posa desiderata
        desired_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/desired_ee_global_pose", rclcpp::QoS(1).transient_local(),
            std::bind(&RvizMarkerNode::desired_pose_callback, this, std::placeholders::_1));

        // Subscriber per la posa attuale
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/ee_world_pose", 10,
            std::bind(&RvizMarkerNode::current_pose_callback, this, std::placeholders::_1));

        // Publisher per i marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pose_markers", 
            rclcpp::QoS(1).transient_local());

        }

private:
    void desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        publish_marker(*msg, "desired_pose", 0, 0.0, 1.0, 0.0); // Verde
    }

    void current_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        publish_marker(*msg, "current_pose", 1, 1.0, 0.0, 0.0); // Rosso
    }

    void publish_marker(const geometry_msgs::msg::Pose& pose, const std::string& ns, int id, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = ns; //namespace
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;

    rclcpp::TimerBase::SharedPtr transform_timer_;

    bool use_gazebo_pose_;
    bool has_vehicle_local_position_;
    bool has_vehicle_attitude_;
    geometry_msgs::msg::Pose vehicle_local_position_;
    geometry_msgs::msg::Pose vehicle_attitude_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
