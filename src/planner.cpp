#include "clik1_node_pkg/planner.hpp"
#include <string>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <chrono>
#include <cmath>


PlannerNode::PlannerNode() : Node("planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
  RCLCPP_INFO(this->get_logger(), "Nodo planner avviato.");

  this->declare_parameter<bool>("use_gazebo_pose", true);
  this->get_parameter("use_gazebo_pose", use_gazebo_pose_);

  // Carica il modello URDF (stesso del clik_uam_node per ottenere trasformazioni statiche)
  const auto pkg_share = ament_index_cpp::get_package_share_directory("clik1_node_pkg");
  const std::string urdf_filename = pkg_share + "/model/t960a.urdf";
  try {
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento del modello URDF: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  if (!model_.existFrame("mobile_wx250s/ee_gripper_link")) {
    RCLCPP_ERROR(this->get_logger(), "Frame 'mobile_wx250s/ee_gripper_link' mancante nel modello.");
    rclcpp::shutdown();
    return;
  }
  ee_frame_id_ = model_.getFrameId("mobile_wx250s/ee_gripper_link");

  if (use_gazebo_pose_) {
    gazebo_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/world/default/dynamic_pose/info", 10, std::bind(&PlannerNode::gazebo_pose_callback, this, std::placeholders::_1));
  } else {
    real_drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/real_t960a_pose", 10, std::bind(&PlannerNode::real_drone_pose_callback, this, std::placeholders::_1));
  }

  desired_ee_global_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
    "/desired_ee_global_pose", rclcpp::QoS(10));
  desired_ee_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/desired_ee_velocity", rclcpp::QoS(10));
}

void PlannerNode::run() {
  while (rclcpp::ok()) {
    int option = 0;
    while (rclcpp::ok()) {
      std::cout << "What do you want the end-effector to do?" << std::endl;
      std::cout << "1. Positioning" << std::endl;
      std::cout << "2. Circular trajectory (x-z plane)" << std::endl;
      std::cout << "> ";
      std::string input; std::getline(std::cin, input);
      try { option = std::stoi(input); } catch (...) { option = 0; }
      if (option == 1 || option == 2) break;
      std::cout << "Opzione non valida. Riprova." << std::endl;
    }
    if (!rclcpp::ok()) break;

    if (option == 1) {
      get_and_transform_desired_pose();
    } else if (option == 2) {
      run_circular_trajectory();
    }
  }
}

// --- Funzioni copiate identiche da clik_uam_node.cpp ---
void PlannerNode::get_and_transform_desired_pose() {
    // Aggiorna le subscription prima di acquisire input (può arrivare nuova posa drone)
    rclcpp::spin_some(this->get_node_base_interface());
    geometry_msgs::msg::Pose desired_pose_local;
    std::string input;

    RCLCPP_INFO(this->get_logger(), "Inserire la posa desiderata per l'end-effector (rispetto a 'mobile_wx250s/base_link').");
    RCLCPP_INFO(this->get_logger(), "Formato: 'x y z qx qy qz qw' (7 valori separati da spazio).");
    RCLCPP_INFO(this->get_logger(), "Premere INVIO per usare la posa di default [0.45 0 0.36 0 0 0 1].");
    std::cout << "> ";
    std::getline(std::cin, input);

    std::stringstream ss(input);
    double val;
    std::vector<double> values;
    while (ss >> val)
    {
        values.push_back(val);
    }

    if (values.size() == 7)
    {
        desired_pose_local.position.x = values[0];
        desired_pose_local.position.y = values[1];
        desired_pose_local.position.z = values[2];
        desired_pose_local.orientation.x = values[3];
        desired_pose_local.orientation.y = values[4];
        desired_pose_local.orientation.z = values[5];
        desired_pose_local.orientation.w = values[6];
        RCLCPP_INFO(this->get_logger(), "Posa desiderata impostata dall'utente.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Input non valido o assente. Calcolo della posa di default (home) con Pinocchio.");

        pinocchio::Data data_home(model_);
        const Eigen::VectorXd q_home = pinocchio::neutral(model_);
        pinocchio::framesForwardKinematics(model_, data_home, q_home);
        const pinocchio::FrameIndex arm_base_frame_id = model_.getFrameId("mobile_wx250s/base_link");
        const pinocchio::SE3& T_world_ee = data_home.oMf[ee_frame_id_];
        const pinocchio::SE3& T_world_arm_base = data_home.oMf[arm_base_frame_id];
        const pinocchio::SE3 T_arm_base_ee = T_world_arm_base.inverse() * T_world_ee;
        desired_pose_local.position.x = T_arm_base_ee.translation().x();
        desired_pose_local.position.y = T_arm_base_ee.translation().y();
        desired_pose_local.position.z = T_arm_base_ee.translation().z();
        Eigen::Quaterniond q_local(T_arm_base_ee.rotation());
        desired_pose_local.orientation.x = q_local.x();
        desired_pose_local.orientation.y = q_local.y();
        desired_pose_local.orientation.z = q_local.z();
        desired_pose_local.orientation.w = q_local.w();
    }

    this->desired_ee_pose_local_ = desired_pose_local;

    RCLCPP_INFO(this->get_logger(), "Posa desiderata (locale): x=%.2f, y=%.2f, z=%.2f, qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f",
                desired_ee_pose_local_.position.x, desired_ee_pose_local_.position.y, desired_ee_pose_local_.position.z,
                desired_ee_pose_local_.orientation.x, desired_ee_pose_local_.orientation.y, desired_ee_pose_local_.orientation.z, desired_ee_pose_local_.orientation.w);

    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok() && (!has_vehicle_local_position_ || !has_vehicle_attitude_))
    {
        RCLCPP_WARN(this->get_logger(), "In attesa dei dati di posizione e assetto del veicolo...");
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }

    if (!has_vehicle_local_position_ || !has_vehicle_attitude_)
    {
        RCLCPP_ERROR(this->get_logger(), "Posizione o assetto del veicolo non disponibili. Impossibile calcolare la posa globale dell'EE.");
        return;
    }

    // Aggiorna ancora per avere i dati più freschi prima della trasformazione
    rclcpp::spin_some(this->get_node_base_interface());
    pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::FrameIndex frame_id = model_.getFrameId("mobile_wx250s/base_link");
    const pinocchio::SE3& tf_base_to_arm_base = data_.oMf[frame_id];

    geometry_msgs::msg::Pose drone_pose;
    drone_pose.position.x = vehicle_local_position_.x;
    drone_pose.position.y = vehicle_local_position_.y;
    drone_pose.position.z = vehicle_local_position_.z;
    drone_pose.orientation.x = vehicle_attitude_.q[1];
    drone_pose.orientation.y = vehicle_attitude_.q[2];
    drone_pose.orientation.z = vehicle_attitude_.q[3];
    drone_pose.orientation.w = vehicle_attitude_.q[0];

    tf2::Transform tf_drone_pose;
    tf2::fromMsg(drone_pose, tf_drone_pose);

    tf2::Transform tf_arm_base_to_local_pose;
    tf2::fromMsg(desired_ee_pose_local_, tf_arm_base_to_local_pose);

    tf2::Transform tf_base_to_arm_base_tf2;
    tf_base_to_arm_base_tf2.setOrigin(tf2::Vector3(tf_base_to_arm_base.translation().x(), tf_base_to_arm_base.translation().y(), tf_base_to_arm_base.translation().z()));
    Eigen::Quaterniond eigen_quat(tf_base_to_arm_base.rotation());
    tf2::Quaternion tf2_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    tf_base_to_arm_base_tf2.setRotation(tf2_quat);

    tf2::Transform tf_world_to_desired_pose = tf_drone_pose * tf_base_to_arm_base_tf2 * tf_arm_base_to_local_pose;
    tf2::toMsg(tf_world_to_desired_pose, desired_ee_pose_world_);

    RCLCPP_INFO(this->get_logger(), "Posa desiderata (world): x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                desired_ee_pose_world_.position.x, desired_ee_pose_world_.position.y, desired_ee_pose_world_.position.z,
                desired_ee_pose_world_.orientation.x, desired_ee_pose_world_.orientation.y, desired_ee_pose_world_.orientation.z, desired_ee_pose_world_.orientation.w);

    // Pubblica sempre la posa
    desired_ee_global_pose_pub_->publish(desired_ee_pose_world_);
    // In modalità positioning la velocità deve essere nulla
    if (desired_ee_velocity_pub_) {
      geometry_msgs::msg::Twist zero_twist; desired_ee_velocity_pub_->publish(zero_twist);
    }
    last_published_pose_ = desired_ee_pose_world_;
    desired_ee_pose_world_ready_ = true;
}

void PlannerNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  // Conversione da NED (PX4) a FLU (Forward, Left, Up)
  vehicle_local_position_ = *msg; // copia originale
  vehicle_local_position_.x = msg->y;      // Forward (North)
  vehicle_local_position_.y = msg->x;     // Left (negazione di East)
  vehicle_local_position_.z = -msg->z;     // Up (negazione di Down)
  has_vehicle_local_position_ = true;
}

void PlannerNode::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  Eigen::Quaterniond flu_quat(msg->q[0], msg->q[1], -msg->q[2], -msg->q[3]);
  vehicle_attitude_ = *msg;
  vehicle_attitude_.q[0] = flu_quat.w();
  vehicle_attitude_.q[1] = flu_quat.x();
  vehicle_attitude_.q[2] = flu_quat.y();
  vehicle_attitude_.q[3] = flu_quat.z();
  has_vehicle_attitude_ = true;
}

void PlannerNode::real_drone_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  vehicle_local_position_.x = msg->position.x;
  vehicle_local_position_.y = msg->position.y;
  vehicle_local_position_.z = msg->position.z;
  vehicle_attitude_.q[0] = msg->orientation.w;
  vehicle_attitude_.q[1] = msg->orientation.x;
  vehicle_attitude_.q[2] = msg->orientation.y;
  vehicle_attitude_.q[3] = msg->orientation.z;
  has_vehicle_local_position_ = true;
  has_vehicle_attitude_ = true;
}

void PlannerNode::gazebo_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  if (!msg->poses.empty()) {
    const auto& pose = msg->poses[0];
    vehicle_local_position_.x = pose.position.x;
    vehicle_local_position_.y = pose.position.y;
    vehicle_local_position_.z = pose.position.z;
    vehicle_attitude_.q[0] = pose.orientation.w;
    vehicle_attitude_.q[1] = pose.orientation.x;
    vehicle_attitude_.q[2] = pose.orientation.y;
    vehicle_attitude_.q[3] = pose.orientation.z;
    has_vehicle_local_position_ = true;
    has_vehicle_attitude_ = true;
  }
}

void PlannerNode::publish_desired_global_pose(const geometry_msgs::msg::Pose& pose) {
  // Pubblica sempre (rimosso controllo di uguaglianza)
  desired_ee_global_pose_pub_->publish(pose);
  last_published_pose_ = pose;
}

void PlannerNode::run_circular_trajectory() {
  // Parametri input utente
  std::cout << "Inserire POSIZIONE EE DI PARTENZA (x y z) in metri rispetto a mobile_wx250s/base_link [default 0.3 0.0 0.36]:\n> ";
  std::string input; std::getline(std::cin, input);
  std::stringstream ss(input);
  std::vector<double> p0v; double v; while (ss >> v) p0v.push_back(v);
  Eigen::Vector3d p0;
  if (p0v.size() == 3) p0 = Eigen::Vector3d(p0v[0], p0v[1], p0v[2]);
  else p0 = Eigen::Vector3d(0.3, 0.0, 0.36);

  std::cout << "Inserire RAGGIO della circonferenza in cm:\n> ";
  std::getline(std::cin, input);
  double r_cm = 0.0; try { r_cm = std::stod(input); } catch (...) { r_cm = 0.0; }
  double R = r_cm / 100.0; // metri
  if (R <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Raggio non valido. Imposto R=0.08 m");
    R = 0.08;
  }

  std::cout << "Inserire TEMPO DI PERCORRENZA totale in secondi:\n> ";
  std::getline(std::cin, input);
  double T = 0.0; try { T = std::stod(input); } catch (...) { T = 0.0; }
  if (T <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Tempo non valido. Imposto T=10.0 s");
    T = 10.0;
  }

  // Attendi pose drone
  rclcpp::Rate wait_rate(10);
  while (rclcpp::ok() && (!has_vehicle_local_position_ || !has_vehicle_attitude_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "In attesa della posa del drone...");
    rclcpp::spin_some(this->get_node_base_interface());
    wait_rate.sleep();
  }
  if (!rclcpp::ok()) return;

  // Prepara trasformazioni statiche base_link -> arm_base e orientazione iniziale EE
  pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::FrameIndex arm_base_frame_id = model_.getFrameId("mobile_wx250s/base_link");
  const pinocchio::SE3& T_world_arm_base = data_.oMf[arm_base_frame_id];

  // orientazione EE da mantenere: useremo quella del primo setpoint pubblicato
  Eigen::Quaterniond q_world_ee(1.0, 0.0, 0.0, 0.0); // placeholder, sarà aggiornato dopo il primo toMsg

  // Converte p0 (locale base_link) in world e pubblica come setpoint iniziale di posizione (vel=0)
  tf2::Transform tf_base_to_arm_base;
  tf_base_to_arm_base.setOrigin({T_world_arm_base.translation().x(), T_world_arm_base.translation().y(), T_world_arm_base.translation().z()});
  Eigen::Quaterniond q_base(T_world_arm_base.rotation());
  tf2::Quaternion q_tf(q_base.x(), q_base.y(), q_base.z(), q_base.w());
  tf_base_to_arm_base.setRotation(q_tf);

  geometry_msgs::msg::Pose start_pose_local;
  start_pose_local.position.x = p0.x();
  start_pose_local.position.y = p0.y();
  start_pose_local.position.z = p0.z();
  start_pose_local.orientation.x = q_world_ee.x();
  start_pose_local.orientation.y = q_world_ee.y();
  start_pose_local.orientation.z = q_world_ee.z();
  start_pose_local.orientation.w = q_world_ee.w();

  tf2::Transform tf_arm_base_to_local_pose; tf2::fromMsg(start_pose_local, tf_arm_base_to_local_pose);

  geometry_msgs::msg::Pose drone_pose;
  drone_pose.position.x = vehicle_local_position_.x;
  drone_pose.position.y = vehicle_local_position_.y;
  drone_pose.position.z = vehicle_local_position_.z;
  drone_pose.orientation.x = vehicle_attitude_.q[1];
  drone_pose.orientation.y = vehicle_attitude_.q[2];
  drone_pose.orientation.z = vehicle_attitude_.q[3];
  drone_pose.orientation.w = vehicle_attitude_.q[0];
  tf2::Transform tf_drone_pose; tf2::fromMsg(drone_pose, tf_drone_pose);

  // Congela la trasformazione world <- arm_base all'istante di avvio
  tf2::Transform tf_world_from_arm_base0 = tf_drone_pose * tf_base_to_arm_base;

  tf2::Transform tf_world_to_start = tf_world_from_arm_base0 * tf_arm_base_to_local_pose;
  geometry_msgs::msg::Pose start_pose_world; tf2::toMsg(tf_world_to_start, start_pose_world);

  // Aggiorna orientazione di riferimento come quella del primo setpoint world
  q_world_ee = Eigen::Quaterniond(start_pose_world.orientation.w, start_pose_world.orientation.x, start_pose_world.orientation.y, start_pose_world.orientation.z);

  // Pubblica pose iniziale e velocità zero
  desired_ee_global_pose_pub_->publish(start_pose_world);
  geometry_msgs::msg::Twist zero_twist; desired_ee_velocity_pub_->publish(zero_twist);

  RCLCPP_INFO(this->get_logger(), "Avvio traiettoria circolare: R=%.3f m, T=%.2f s", R, T);

  // Centro della circonferenza sul piano x-z davanti (x maggiore) alla stessa quota z di p0; scegliamo centro = p0 + [R, 0, 0]
  Eigen::Vector3d c_local = p0 + Eigen::Vector3d(R, 0.0, 0.0);

  // Profilo tempo con S-curve semplice (trapezoidale acc+dec con velocità angolare omega costante):
  // Per semplicità: profilo cosenoide su 0..T: s(t) = 0.5*(1 - cos(pi * t / T)) in [0,1];
  // Parametrizza angolo theta(t) = 2*pi * s(t); velocità angolare = dtheta/dt = 2*pi * ds/dt
  auto now = this->now();
  rclcpp::Time t0 = now;
  rclcpp::Rate rate(100); // 100 Hz
  geometry_msgs::msg::Pose pose_msg;
  geometry_msgs::msg::Twist vel_msg;

  while (rclcpp::ok()) {
    rclcpp::Time t = this->now();
    double tau = (t - t0).seconds();
    if (tau > T) break;

    double s = 0.5 * (1.0 - std::cos(M_PI * tau / T)); // 0->1, ds/dt = 0 a t=0,T
    double dsdt = 0.5 * (M_PI / T) * std::sin(M_PI * tau / T);
    double theta = 2.0 * M_PI * s;            // 0 -> 2pi
    double dtheta = 2.0 * M_PI * dsdt;        // derivata

    // Punto sulla circonferenza nel piano x-z locale (y invariata)
    Eigen::Vector3d p_local;
    p_local.x() = c_local.x() + R * std::cos(theta);
    p_local.y() = p0.y();
    p_local.z() = c_local.z() + R * std::sin(theta);
    // Velocità lineare locale (derivata) nel piano x-z
    Eigen::Vector3d v_local;
    v_local.x() = -R * std::sin(theta) * dtheta;
    v_local.y() = 0.0;
    v_local.z() =  R * std::cos(theta) * dtheta;

  // Costruisci trasformazioni per convertire in world utilizzando la trasformazione congelata
    tf2::Vector3 p_local_tf(p_local.x(), p_local.y(), p_local.z());
    tf2::Vector3 v_local_tf(v_local.x(), v_local.y(), v_local.z());

  tf2::Vector3 p_world_tf = tf_world_from_arm_base0 * p_local_tf;
  tf2::Vector3 v_world_tf = tf_world_from_arm_base0.getBasis() * v_local_tf; // sola rotazione congelata

    pose_msg.position.x = p_world_tf.x();
    pose_msg.position.y = p_world_tf.y();
    pose_msg.position.z = p_world_tf.z();
    pose_msg.orientation.x = q_world_ee.x();
    pose_msg.orientation.y = q_world_ee.y();
    pose_msg.orientation.z = q_world_ee.z();
    pose_msg.orientation.w = q_world_ee.w();

    vel_msg.linear.x = v_world_tf.x();
    vel_msg.linear.y = v_world_tf.y();
    vel_msg.linear.z = v_world_tf.z();
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;

    desired_ee_global_pose_pub_->publish(pose_msg);
    desired_ee_velocity_pub_->publish(vel_msg);

    rate.sleep();
  }

  // Fine traiettoria: pubblica velocità nulla e ultima posa per fermo dolce
  geometry_msgs::msg::Twist zero; desired_ee_velocity_pub_->publish(zero);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  // std::thread spin_thread([&]() { rclcpp::spin(node); });
  node->run();
  rclcpp::shutdown();
  // spin_thread.join();
  return 0;
}
