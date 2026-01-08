#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// Nodo: real_drone_vel_pub
// Funzione: pubblica /real_t960a_twist (geometry_msgs/Twist)
//  - velocità lineare nel frame WORLD-FLU fissato alla direzione di marcia iniziale
//  - velocità angolare nel frame body FLU
// Usa:
//  - /fmu/out/vehicle_odometry per ricavare yaw iniziale (dal quaternione),
//    velocità lineare (NED) e angolare (body FRD)

class RealDroneVelPub : public rclcpp::Node {
public:
  RealDroneVelPub() : Node("real_drone_vel_pub") {
    using std::placeholders::_1;
    RCLCPP_INFO(get_logger(), "Avvio real_drone_vel_pub");

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/real_t960a_twist", 10);

    vehicle_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&RealDroneVelPub::vehicle_odom_cb, this, _1));
  }

private:
  void vehicle_odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    vehicle_odom_ = *msg;
    has_odom_ = true;

    // Inizializza una sola volta lo yaw di riferimento dal quaternione di odometry
    if (!yaw_offset_initialized_) {
      // PX4 VehicleOdometry.q è NED->FRD (w,x,y,z)
      Eigen::Quaterniond q_px4(vehicle_odom_.q[0], vehicle_odom_.q[1], vehicle_odom_.q[2], vehicle_odom_.q[3]);
      // Converti FRD -> FLU: (w, x, -y, -z)
      Eigen::Quaterniond q_flu(q_px4.w(), q_px4.x(), -q_px4.y(), -q_px4.z());
      q_flu.normalize();
      // Estrai yaw dal quaternione FLU
      Eigen::Vector3d eul = q_flu.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
      yaw_offset_ = eul[0];
      yaw_offset_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Yaw iniziale (odometry) catturato: %.3f rad", yaw_offset_);
    }
    try_publish();
  }

  void try_publish() {
    if (!(has_odom_ && yaw_offset_initialized_)) {
      return;
    }

    // Velocità lineare: campo velocity in NED [N, E, D]
    const double x_n = static_cast<double>(vehicle_odom_.velocity[0]);
    const double y_e = static_cast<double>(vehicle_odom_.velocity[1]);
    const double z_d = static_cast<double>(vehicle_odom_.velocity[2]);

    const double cos_y0 = std::cos(yaw_offset_);
    const double sin_y0 = std::sin(yaw_offset_);

    // Rotazione SOLO nel piano orizzontale per fissare il frame WORLD-FLU
    // X_world = forward rispetto heading iniziale
    // Y_world = left rispetto heading iniziale
    const double x_world = cos_y0 * x_n + sin_y0 * y_e;
    const double y_world = sin_y0 * x_n - cos_y0 * y_e;
    const double z_world = -z_d; // Down -> Up

    // Velocità angolare: campo angular_velocity in frame body FRD
    // Conversione FRD (Forward, Right, Down) -> FLU (Forward, Left, Up)
    const double wx_flu = static_cast<double>(vehicle_odom_.angular_velocity[0]);
    const double wy_flu = -static_cast<double>(vehicle_odom_.angular_velocity[1]);
    const double wz_flu = -static_cast<double>(vehicle_odom_.angular_velocity[2]);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = x_world;
    twist.linear.y = y_world;
    twist.linear.z = z_world;
    twist.angular.x = wx_flu;
    twist.angular.y = wy_flu;
    twist.angular.z = wz_flu;

    twist_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;

  px4_msgs::msg::VehicleOdometry vehicle_odom_;
  bool has_odom_{false};
  bool yaw_offset_initialized_{false};
  double yaw_offset_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealDroneVelPub>());
  rclcpp::shutdown();
  return 0;
}
