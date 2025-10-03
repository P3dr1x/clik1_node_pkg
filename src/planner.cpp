#include "clik1_node_pkg/planner.hpp"
#include <string>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <chrono>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"


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
    real_drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/t960a/pose", 10, std::bind(&PlannerNode::real_drone_pose_callback, this, std::placeholders::_1));
  }

  desired_ee_global_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
    "/desired_ee_global_pose", rclcpp::QoS(10));
  desired_ee_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/desired_ee_velocity", rclcpp::QoS(10));

  // Joint states per ricavare la posa corrente EE via Pinocchio
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&PlannerNode::joint_state_callback, this, std::placeholders::_1));
}

void PlannerNode::run() {
  while (rclcpp::ok()) {
    int option = 0;
    while (rclcpp::ok()) {
      std::cout << "What do you want the end-effector to do?" << std::endl;
      std::cout << "1. Positioning" << std::endl;
      std::cout << "2. Circular trajectory (x-z plane)" << std::endl;
      std::cout << "3. Polyline trajectory" << std::endl;
      std::cout << "> ";
      std::string input; std::getline(std::cin, input);
      try { option = std::stoi(input); } catch (...) { option = 0; }
      if (option == 1 || option == 2 || option == 3) break;
      std::cout << "Opzione non valida. Riprova." << std::endl;
    }
    if (!rclcpp::ok()) break;

    if (option == 1) {
      get_and_transform_desired_pose();
    } else if (option == 2) {
      run_circular_trajectory();
    } else if (option == 3) {
      run_polyline_trajectory();
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

void PlannerNode::real_drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto& p = msg->pose.position;
  const auto& o = msg->pose.orientation;
  vehicle_local_position_.x = p.x;
  vehicle_local_position_.y = p.y;
  vehicle_local_position_.z = p.z;
  vehicle_attitude_.q[0] = o.w;
  vehicle_attitude_.q[1] = o.x;
  vehicle_attitude_.q[2] = o.y;
  vehicle_attitude_.q[3] = o.z;
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
  std::cout << "Inserire POSIZIONE EE DI PARTENZA (x y z) in metri rispetto a mobile_wx250s/base_link" << std::endl;
  std::cout << "Premi INVIO per usare la POSIZIONE CORRENTE dell'EE" << std::endl << "> ";
  std::string input; std::getline(std::cin, input);
  std::stringstream ss(input);
  std::vector<double> p0v; double v; while (ss >> v) p0v.push_back(v);
  Eigen::Vector3d p0;
  bool use_current_pose = p0v.empty();
  if (!use_current_pose && p0v.size() == 3) {
    p0 = Eigen::Vector3d(p0v[0], p0v[1], p0v[2]);
  }

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

  // Costruisci trasformazioni base necessarie (arm_base in world, drone in world)
  tf2::Transform tf_base_to_arm_base;
  tf_base_to_arm_base.setOrigin({T_world_arm_base.translation().x(), T_world_arm_base.translation().y(), T_world_arm_base.translation().z()});
  Eigen::Quaterniond q_base(T_world_arm_base.rotation());
  tf2::Quaternion q_tf(q_base.x(), q_base.y(), q_base.z(), q_base.w());
  tf_base_to_arm_base.setRotation(q_tf);

  geometry_msgs::msg::Pose drone_pose;
  drone_pose.position.x = vehicle_local_position_.x;
  drone_pose.position.y = vehicle_local_position_.y;
  drone_pose.position.z = vehicle_local_position_.z;
  drone_pose.orientation.x = vehicle_attitude_.q[1];
  drone_pose.orientation.y = vehicle_attitude_.q[2];
  drone_pose.orientation.z = vehicle_attitude_.q[3];
  drone_pose.orientation.w = vehicle_attitude_.q[0];
  tf2::Transform tf_drone_pose; tf2::fromMsg(drone_pose, tf_drone_pose);

  // Se richiesto, usa la posa corrente dell'EE come punto di partenza
  if (use_current_pose) {
    // Attendi di avere joint states
    rclcpp::Rate r(50);
    for (int i = 0; i < 50 && rclcpp::ok() && !has_joint_state_; ++i) {
      rclcpp::spin_some(this->get_node_base_interface());
      r.sleep();
    }
    if (!has_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "Joint states non disponibili, uso default [0.3,0,0.36]");
      p0 = Eigen::Vector3d(0.3, 0.0, 0.36);
    } else {
      // Costruisci configurazione q per Pinocchio: free-flyer (base) + giunti manipolatore
      Eigen::VectorXd q = pinocchio::neutral(model_);
      // Imposta base con posa attuale del drone
      q[0] = vehicle_local_position_.x;
      q[1] = vehicle_local_position_.y;
      q[2] = vehicle_local_position_.z;
      q[3] = vehicle_attitude_.q[1];
      q[4] = vehicle_attitude_.q[2];
      q[5] = vehicle_attitude_.q[3];
      q[6] = vehicle_attitude_.q[0];
      // Inserisci posizioni giunti disponibili
      for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
        const auto& jname = current_joint_state_.name[i];
        if (!model_.existJointName(jname)) continue;
        pinocchio::JointIndex jid = model_.getJointId(jname);
        int idx_q = static_cast<int>(model_.joints[jid].idx_q());
        if (idx_q >= 7 && idx_q < q.size()) {
          q[idx_q] = current_joint_state_.position[i];
        }
      }
      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);
      const pinocchio::SE3& T_world_ee_now = data_.oMf[ee_frame_id_];
      // Converti world EE -> locale base_link all’avvio: p_local = (world_from_arm_base0)^{-1} * p_world
      tf2::Transform tf_world_from_arm_base0_tmp = tf_drone_pose * tf_base_to_arm_base;
      tf2::Vector3 p_world(T_world_ee_now.translation().x(), T_world_ee_now.translation().y(), T_world_ee_now.translation().z());
      tf2::Transform inv = tf_world_from_arm_base0_tmp.inverse();
      tf2::Vector3 p_local_tf = inv * p_world;
      p0 = Eigen::Vector3d(p_local_tf.x(), p_local_tf.y(), p_local_tf.z());
      RCLCPP_INFO(this->get_logger(), "Start dal corrente EE (locale, FK): [%.3f %.3f %.3f]", p0.x(), p0.y(), p0.z());
    }
  }

  // Converte p0 (locale base_link) in world e pubblica come setpoint iniziale di posizione (vel=0)
  geometry_msgs::msg::Pose start_pose_local;
  start_pose_local.position.x = p0.x();
  start_pose_local.position.y = p0.y();
  start_pose_local.position.z = p0.z();
  start_pose_local.orientation.x = q_world_ee.x();
  start_pose_local.orientation.y = q_world_ee.y();
  start_pose_local.orientation.z = q_world_ee.z();
  start_pose_local.orientation.w = q_world_ee.w();

  tf2::Transform tf_arm_base_to_local_pose; tf2::fromMsg(start_pose_local, tf_arm_base_to_local_pose);

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
  Eigen::Vector3d c_local = p0 - Eigen::Vector3d(R, 0.0, 0.0);

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

void PlannerNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  current_joint_state_ = *msg;
  has_joint_state_ = true;
}

void PlannerNode::run_polyline_trajectory() {
  // 1) Acquisizione waypoints locali rispetto a mobile_wx250s/base_link
  std::cout << "Inserisci sequenza di waypoints. Ogni riga: \n"
               " - 3 numeri: x y z (posizione)\n"
               " - 7 numeri: x y z qx qy qz qw (posa)\n"
               "Termina con una riga vuota (solo INVIO).\n";

  struct WP { Eigen::Vector3d p; bool has_q; Eigen::Quaterniond q; };
  std::vector<WP> wps_local;
  while (rclcpp::ok()) {
    std::cout << "> ";
    std::string line; if (!std::getline(std::cin, line)) break;
    if (line.empty()) break;
    std::stringstream ss(line);
    std::vector<double> vals; double x; while (ss >> x) vals.push_back(x);
    if (vals.size() == 3) {
      wps_local.push_back({Eigen::Vector3d(vals[0], vals[1], vals[2]), false, Eigen::Quaterniond(1,0,0,0)});
    } else if (vals.size() == 7) {
      Eigen::Quaterniond q(vals[6], vals[3], vals[4], vals[5]);
      q.normalize();
      wps_local.push_back({Eigen::Vector3d(vals[0], vals[1], vals[2]), true, q});
    } else {
      std::cout << "Input non valido. Inserire 3 o 7 valori." << std::endl;
    }
  }
  if (wps_local.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Servono almeno 2 waypoints per una polyline.");
    return;
  }

  // 2) Tempo massimo per ciascun tratto
  std::cout << "Tempo massimo per ciascun tratto [s] (default 5.0):\n> ";
  std::string input; std::getline(std::cin, input);
  double Tseg = 5.0; try { if (!input.empty()) Tseg = std::stod(input); } catch (...) {}
  if (Tseg <= 0.0) Tseg = 5.0;

  // 3) Attesa posa drone
  rclcpp::Rate wait_rate(10);
  while (rclcpp::ok() && (!has_vehicle_local_position_ || !has_vehicle_attitude_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "In attesa della posa del drone...");
    rclcpp::spin_some(this->get_node_base_interface());
    wait_rate.sleep();
  }
  if (!rclcpp::ok()) return;

  // 4) Trasformazioni statiche base_link -> arm_base e freeze world<-arm_base all'avvio
  pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::FrameIndex arm_base_frame_id = model_.getFrameId("mobile_wx250s/base_link");
  const pinocchio::SE3& T_world_arm_base = data_.oMf[arm_base_frame_id];

  tf2::Transform tf_base_to_arm_base;
  tf_base_to_arm_base.setOrigin({T_world_arm_base.translation().x(), T_world_arm_base.translation().y(), T_world_arm_base.translation().z()});
  Eigen::Quaterniond q_base(T_world_arm_base.rotation());
  tf2::Quaternion q_tf(q_base.x(), q_base.y(), q_base.z(), q_base.w());
  tf_base_to_arm_base.setRotation(q_tf);

  geometry_msgs::msg::Pose drone_pose;
  drone_pose.position.x = vehicle_local_position_.x;
  drone_pose.position.y = vehicle_local_position_.y;
  drone_pose.position.z = vehicle_local_position_.z;
  drone_pose.orientation.x = vehicle_attitude_.q[1];
  drone_pose.orientation.y = vehicle_attitude_.q[2];
  drone_pose.orientation.z = vehicle_attitude_.q[3];
  drone_pose.orientation.w = vehicle_attitude_.q[0];
  tf2::Transform tf_drone_pose; tf2::fromMsg(drone_pose, tf_drone_pose);
  tf2::Transform tf_world_from_arm_base0 = tf_drone_pose * tf_base_to_arm_base; // frozen

  // 5) Pubblica il primo waypoint come posa iniziale e vel zero
  auto to_world_pose = [&](const WP& wp)->geometry_msgs::msg::Pose{
    // locale -> world usando frozen transform
    tf2::Transform T_local;
    tf2::Vector3 p(wp.p.x(), wp.p.y(), wp.p.z());
    T_local.setOrigin(p);
    if (wp.has_q) {
      tf2::Quaternion q(wp.q.x(), wp.q.y(), wp.q.z(), wp.q.w());
      T_local.setRotation(q);
    } else {
      T_local.setRotation(tf2::Quaternion(0,0,0,1));
    }
    tf2::Transform T_world = tf_world_from_arm_base0 * T_local;
    geometry_msgs::msg::Pose out; tf2::toMsg(T_world, out);
    return out;
  };

  // Orientazione di riferimento: se il primo WP ha quaternione usalo, altrimenti prendi quella world del primo setpoint
  Eigen::Quaterniond q_world_ref(1,0,0,0);
  geometry_msgs::msg::Pose first_pose_world = to_world_pose(wps_local.front());
  if (wps_local.front().has_q) {
    q_world_ref = Eigen::Quaterniond(first_pose_world.orientation.w, first_pose_world.orientation.x, first_pose_world.orientation.y, first_pose_world.orientation.z);
  } else {
    q_world_ref = Eigen::Quaterniond(first_pose_world.orientation.w, first_pose_world.orientation.x, first_pose_world.orientation.y, first_pose_world.orientation.z);
  }
  desired_ee_global_pose_pub_->publish(first_pose_world);
  geometry_msgs::msg::Twist zero; desired_ee_velocity_pub_->publish(zero);

  rclcpp::Rate rate(100); // 100 Hz
  RCLCPP_INFO(this->get_logger(), "Avvio polyline con %zu waypoints, T per tratto = %.2f s", wps_local.size(), Tseg);

  auto scurve = [&](double t, double T){
    double s = 0.5 * (1.0 - std::cos(M_PI * t / T));
    double dsdt = 0.5 * (M_PI / T) * std::sin(M_PI * t / T);
    return std::pair<double,double>(s, dsdt);
  };

  for (size_t i = 0; i + 1 < wps_local.size() && rclcpp::ok(); ++i) {
    const WP& A = wps_local[i];
    const WP& B = wps_local[i+1];
    // traiettoria locale lineare p(t) = A + s*(B-A)
    Eigen::Vector3d d = B.p - A.p;

    // orientazione world target per il segmento
    Eigen::Quaterniond q_target_world = q_world_ref;
    bool use_orient = false;
    if (A.has_q && B.has_q) {
      // mantieni orientazione lungo SLERP tra A e B in world, convertendo i quaternioni locali in world una volta sola
      geometry_msgs::msg::Pose Aw = to_world_pose(A);
      geometry_msgs::msg::Pose Bw = to_world_pose(B);
      Eigen::Quaterniond qa(Aw.orientation.w, Aw.orientation.x, Aw.orientation.y, Aw.orientation.z);
      Eigen::Quaterniond qb(Bw.orientation.w, Bw.orientation.x, Bw.orientation.y, Bw.orientation.z);
      // useremo slerp(s)
      use_orient = true;
      // q_target_world verrà slerp-ato dentro al loop
      q_target_world = qa; // init
    } else if (B.has_q) {
      geometry_msgs::msg::Pose Bw = to_world_pose(B);
      q_target_world = Eigen::Quaterniond(Bw.orientation.w, Bw.orientation.x, Bw.orientation.y, Bw.orientation.z);
      use_orient = true; // interp da q_world_ref a q_target_world
    } else {
      use_orient = true; // mantieni q_world_ref costante
      q_target_world = q_world_ref;
    }

    rclcpp::Time t0 = this->now();
    while (rclcpp::ok()) {
      double t = (this->now() - t0).seconds();
      if (t > Tseg) break;
      auto [s, dsdt] = scurve(t, Tseg);
      Eigen::Vector3d p_local = A.p + s * d;
      Eigen::Vector3d v_local = dsdt * d;

      // world transform congelata
      tf2::Vector3 p_local_tf(p_local.x(), p_local.y(), p_local.z());
      tf2::Vector3 v_local_tf(v_local.x(), v_local.y(), v_local.z());
      tf2::Vector3 p_world_tf = tf_world_from_arm_base0 * p_local_tf;
      tf2::Vector3 v_world_tf = tf_world_from_arm_base0.getBasis() * v_local_tf;

      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = p_world_tf.x();
      pose_msg.position.y = p_world_tf.y();
      pose_msg.position.z = p_world_tf.z();

      Eigen::Quaterniond q_world = q_world_ref;
      if (use_orient) {
        if (A.has_q && B.has_q) {
          geometry_msgs::msg::Pose Aw = to_world_pose(A);
          geometry_msgs::msg::Pose Bw = to_world_pose(B);
          Eigen::Quaterniond qa(Aw.orientation.w, Aw.orientation.x, Aw.orientation.y, Aw.orientation.z);
          Eigen::Quaterniond qb(Bw.orientation.w, Bw.orientation.x, Bw.orientation.y, Bw.orientation.z);
          q_world = qa.slerp(s, qb);
        } else if (B.has_q && !(A.has_q)) {
          // interp da q_world_ref (inizio segmento) a q_target_world
          q_world = q_world_ref.slerp(s, q_target_world);
        } else {
          q_world = q_world_ref; // costante
        }
      }

      pose_msg.orientation.x = q_world.x();
      pose_msg.orientation.y = q_world.y();
      pose_msg.orientation.z = q_world.z();
      pose_msg.orientation.w = q_world.w();

      geometry_msgs::msg::Twist vel_msg;
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

    // fine segmento: posa finale esatta del waypoint B, velocità zero
    geometry_msgs::msg::Pose B_world = to_world_pose(B);
    // se non hanno orientazioni, imposta orientazione coerente con q_world_ref o mantieni q_world_ref
    if (!B.has_q) {
      B_world.orientation.x = q_world_ref.x();
      B_world.orientation.y = q_world_ref.y();
      B_world.orientation.z = q_world_ref.z();
      B_world.orientation.w = q_world_ref.w();
    }
    desired_ee_global_pose_pub_->publish(B_world);
    geometry_msgs::msg::Twist zero_twist; desired_ee_velocity_pub_->publish(zero_twist);

    // aggiorna orientazione di riferimento per segmento successivo
    if (B.has_q) {
      q_world_ref = Eigen::Quaterniond(B_world.orientation.w, B_world.orientation.x, B_world.orientation.y, B_world.orientation.z);
    } // altrimenti mantieni quella precedente
  }
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
