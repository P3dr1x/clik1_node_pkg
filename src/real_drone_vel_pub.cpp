#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
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

    // Parametri filtro passa-basso (0 => no filtro)
    this->declare_parameter<double>("v_lp_tau", 0.05);     // [s]
    this->declare_parameter<double>("omega_lp_tau", 0.05); // [s]
    v_lp_tau_ = this->get_parameter("v_lp_tau").as_double();
    omega_lp_tau_ = this->get_parameter("omega_lp_tau").as_double();
    RCLCPP_INFO(get_logger(), "v_lp_tau=%.3f s, omega_lp_tau=%.3f s", v_lp_tau_, omega_lp_tau_);

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/real_t960a_twist", 10);

    vehicle_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&RealDroneVelPub::vehicle_odom_cb, this, _1));
  }

private:
  static Eigen::Vector3d lowpassIIR(const Eigen::Vector3d &x_raw, const Eigen::Vector3d &x_prev,
                                   double dt, double tau) {
    if (!(tau > 0.0) || !(dt > 0.0)) {
      return x_raw;
    }
    const double alpha = std::exp(-dt / tau);
    return alpha * x_prev + (1.0 - alpha) * x_raw;
  }

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

    // dt del filtro: usa timestamp PX4 (microsecondi) se disponibile; fallback su now()
    double dt_f = 0.0;
    const uint64_t t_us = static_cast<uint64_t>(vehicle_odom_.timestamp);
    if (t_us != 0u && last_odom_timestamp_us_ != 0u && t_us > last_odom_timestamp_us_) {
      dt_f = 1e-6 * static_cast<double>(t_us - last_odom_timestamp_us_);
    } else if (last_now_init_) {
      dt_f = (this->now() - last_now_).seconds();
    }
    if (dt_f > 0.0) {
      dt_f = std::min(std::max(dt_f, 1e-4), 0.05);
    }
    last_odom_timestamp_us_ = t_us;
    last_now_ = this->now();
    last_now_init_ = true;

    // Velocità lineare: campo velocity in NED [N, E, D]
    Eigen::Vector3d v_raw_ned(
        static_cast<double>(vehicle_odom_.velocity[0]),
        static_cast<double>(vehicle_odom_.velocity[1]),
        static_cast<double>(vehicle_odom_.velocity[2]));

    if (v_filt_init_) {
      v_filt_ned_ = lowpassIIR(v_raw_ned, v_filt_ned_, dt_f, v_lp_tau_);
    } else {
      v_filt_ned_ = v_raw_ned;
      v_filt_init_ = true;
    }

    const double x_n = v_filt_ned_.x();
    const double y_e = v_filt_ned_.y();
    const double z_d = v_filt_ned_.z();

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
    Eigen::Vector3d omega_raw_flu(
        static_cast<double>(vehicle_odom_.angular_velocity[0]),
        -static_cast<double>(vehicle_odom_.angular_velocity[1]),
        -static_cast<double>(vehicle_odom_.angular_velocity[2]));

    if (omega_filt_init_) {
      omega_filt_flu_ = lowpassIIR(omega_raw_flu, omega_filt_flu_, dt_f, omega_lp_tau_);
    } else {
      omega_filt_flu_ = omega_raw_flu;
      omega_filt_init_ = true;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = x_world;
    twist.linear.y = y_world;
    twist.linear.z = z_world;
    twist.angular.x = omega_filt_flu_.x();
    twist.angular.y = omega_filt_flu_.y();
    twist.angular.z = omega_filt_flu_.z();

    twist_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;

  px4_msgs::msg::VehicleOdometry vehicle_odom_;
  bool has_odom_{false};
  bool yaw_offset_initialized_{false};
  double yaw_offset_{0.0};

  // Parametri filtro
  double v_lp_tau_{0.05};
  double omega_lp_tau_{0.05};

  // Stato filtro
  bool v_filt_init_{false};
  Eigen::Vector3d v_filt_ned_{0.0, 0.0, 0.0};
  bool omega_filt_init_{false};
  Eigen::Vector3d omega_filt_flu_{0.0, 0.0, 0.0};

  // Tempo per dt
  uint64_t last_odom_timestamp_us_{0u};
  bool last_now_init_{false};
  rclcpp::Time last_now_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealDroneVelPub>());
  rclcpp::shutdown();
  return 0;
}
