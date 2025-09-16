#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <Eigen/Geometry>

// Nodo: real_drone_pose_pub
// Funzione: pubblica /real_t960a_pose (geometry_msgs/Pose) in frame FLU inizializzato
// con X = direzione di marcia iniziale, Y = sinistra, Z = alto.
// Rimuove yaw iniziale dai dati PX4 (NED) e converte posizione & orientazione.

class RealDronePosePub : public rclcpp::Node {
public:
  RealDronePosePub() : Node("real_drone_pose_pub") {
    using std::placeholders::_1;
    RCLCPP_INFO(get_logger(), "Avvio real_drone_pose_pub");

    pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("/real_t960a_pose", 10);
    vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        std::bind(&RealDronePosePub::vehicle_local_position_cb, this, _1));
    vehicle_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
        std::bind(&RealDronePosePub::vehicle_attitude_cb, this, _1));
  }

private:
  void vehicle_local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    vehicle_local_position_ = *msg; // North, East, Down
    has_position_ = true;
    try_publish();
  }

  void vehicle_attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    vehicle_attitude_ = *msg; // Quaternion NED->FRD (w,x,y,z)
    has_attitude_ = true;
    try_publish();
  }

  double extract_yaw_flu(const Eigen::Quaterniond &q) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2,1,0); // yaw,pitch,roll
    return euler[0];
  }

  void try_publish() {
    if (!(has_position_ && has_attitude_)) return;

    Eigen::Quaterniond q_px4(vehicle_attitude_.q[0], vehicle_attitude_.q[1], vehicle_attitude_.q[2], vehicle_attitude_.q[3]);
    Eigen::Quaterniond q_flu(q_px4.w(), q_px4.x(), -q_px4.y(), -q_px4.z());
    q_flu.normalize();

    // Offset fisso: cattura yaw iniziale una sola volta e poi rimuovilo
    if (!yaw_offset_initialized_) {
      yaw_offset_ = extract_yaw_flu(q_flu);
      yaw_offset_quat_ = Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ()); // rotazione per rimuovere yaw iniziale
      yaw_offset_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Yaw iniziale catturata: %.3f rad", yaw_offset_);
    }
    double yaw_now = extract_yaw_flu(q_flu);
    // Quaternione con yaw relativo: Rz(-yaw_offset_) * q_flu = Rz(yaw_now - yaw_offset_) * Rpr
    Eigen::Quaterniond q_world = yaw_offset_quat_ * q_flu;
    q_world.normalize();

    const double x_n = vehicle_local_position_.x;
    const double y_e = vehicle_local_position_.y;
    const double z_d = vehicle_local_position_.z;
    const double cos_y0 = std::cos(yaw_offset_);
    const double sin_y0 = std::sin(yaw_offset_);
    // Rotazione SOLO della posizione in base allo yaw iniziale per definire frame fisso
    double x_world =  cos_y0 * x_n + sin_y0 * y_e;      // forward rispetto heading iniziale
    double y_world =  sin_y0 * x_n - cos_y0 * y_e;      // left rispetto heading iniziale
    double z_world = -z_d;

    // Offset: il punto originario (P) pubblicato da PX4 è 0.061 m sotto l'origine di base_link lungo +Z corpo (FLU).
    // Quindi base_link = P + R_world_body * (0,0,0.061).
    Eigen::Vector3d offset_body(0.0, 0.0, base_link_offset_z_);
    Eigen::Vector3d offset_world = q_world * offset_body; // q_world ruota vettori dal body al world
    double x_base = x_world + offset_world.x();
    double y_base = y_world + offset_world.y();
    double z_base = z_world + offset_world.z();

    geometry_msgs::msg::Pose pose;
    pose.position.x = x_base;
    pose.position.y = y_base;
    pose.position.z = z_base;
    pose.orientation.w = q_world.w();
    pose.orientation.x = q_world.x();
    pose.orientation.y = q_world.y();
    pose.orientation.z = q_world.z();
    pose_pub_->publish(pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
  px4_msgs::msg::VehicleAttitude vehicle_attitude_;
  bool has_position_{false};
  bool has_attitude_{false};
  bool yaw_offset_initialized_{false};
  double yaw_offset_{0.0};
  Eigen::Quaterniond yaw_offset_quat_{Eigen::Quaterniond::Identity()};
  const double base_link_offset_z_{0.061}; // metri (P -> base_link lungo +Z corpo)
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealDronePosePub>());
  rclcpp::shutdown();
  return 0;
}
