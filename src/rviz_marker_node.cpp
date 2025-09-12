#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <map>

class RvizMarkerNode : public rclcpp::Node {
public:
    RvizMarkerNode() : Node("rviz_marker_node") {
        this->declare_parameter<bool>("real_system", false);
        this->get_parameter("real_system", real_system_);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscriber per la posa desiderata
        desired_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/desired_ee_global_pose", rclcpp::QoS(10),
            std::bind(&RvizMarkerNode::desired_pose_callback, this, std::placeholders::_1));

        // Subscriber per la posa attuale
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/ee_world_pose", 10,
            std::bind(&RvizMarkerNode::current_pose_callback, this, std::placeholders::_1));

        // Publisher per i marker dinamici (current & drone) - volatile
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pose_markers", rclcpp::QoS(10));
        // Publisher dedicato alla posa desiderata - transient_local per recupero dopo avvio RViz
        desired_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/des_ee_pose_marker", rclcpp::QoS(5));

        if (real_system_) {
            // Sottoscrizione al nuovo topic già trasformato
            real_drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "/real_t960a_pose", 10,
                std::bind(&RvizMarkerNode::real_drone_pose_callback, this, std::placeholders::_1));
        }
    }

private:
    void desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        // Pubblica sul topic dedicato
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "desired_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = *msg;
        marker.scale.x = 0.06;
        marker.scale.y = 0.06;
        marker.scale.z = 0.06;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        desired_marker_pub_->publish(marker);
    }

    void current_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        publish_marker(*msg, "current_pose", 1, 1.0f, 0.0f, 0.0f); // Rosso
    }

    void real_drone_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "drone_pose";
        marker.id = 2;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = *msg;
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_pub_->publish(marker);
    }

    void publish_marker(const geometry_msgs::msg::Pose& pose, const std::string& ns, int id, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = ns;
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
        marker.color.a = 1.0f;
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr desired_marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr real_drone_pose_sub_;

    bool real_system_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
