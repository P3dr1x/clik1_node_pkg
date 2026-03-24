#include "clik1_node_pkg/planner.hpp"
#include <string>
#include <algorithm>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <chrono>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace {
int read_positive_int_or_default(const std::string &prompt, int default_value)
{
  std::cout << prompt;
  std::string line;
  if (!std::getline(std::cin, line)) {
    return default_value;
  }
  if (line.empty()) {
    return default_value;
  }
  try {
    const int v = std::stoi(line);
    return std::max(1, v);
  } catch (...) {
    return default_value;
  }
}

double read_double_or_default(const std::string &prompt, double default_value)
{
  std::cout << prompt;
  std::string line;
  if (!std::getline(std::cin, line)) {
    return default_value;
  }
  if (line.empty()) {
    return default_value;
  }
  try {
    return std::stod(line);
  } catch (...) {
    return default_value;
  }
}
} // namespace


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

  // Facoltativo: sottoscrivi alla posa reale dell'EE pubblicata dal controller, se presente
  // Usa una lambda diretta (evita ambiguità di std::bind con functor/lambda e function_traits)
  ee_world_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/ee_world_pose", 10, [this](const geometry_msgs::msg::Pose::SharedPtr msg){
      this->current_ee_pose_ = *msg;
      this->has_current_ee_pose_ = true;
    });

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
      std::cout << "2. Circular trajectory" << std::endl;
      std::cout << "3. Polyline trajectory" << std::endl;
      std::cout << "4. Back-and-forth" << std::endl;
      std::cout << "> ";
      std::string input; std::getline(std::cin, input);
      try { option = std::stoi(input); } catch (...) { option = 0; }
      if (option == 1 || option == 2 || option == 3 || option == 4) break;
      std::cout << "Opzione non valida. Riprova." << std::endl;
    }
    if (!rclcpp::ok()) break;

    if (option == 1) {
      get_and_transform_desired_pose();
    } else if (option == 2) {
      run_circular_trajectory();
    } else if (option == 3) {
      run_polyline_trajectory();
    } else if (option == 4) {
      run_back_and_forth_trajectory();
    }
  }
}

void PlannerNode::run_back_and_forth_trajectory()
{
  // Specifica da planner_additions.md:
  // - scelta piano: sagittale (parallelo a x-z del drone) default oppure frontale (parallelo a y-z del drone)
  // - nel piano sagittale: segmento orizzontale (asse x) con punto medio (0.4, 0, 0.3) nel frame mobile_wx250s/base_link
  // - nel piano frontale: segmento orizzontale (asse y) con punto medio (0.5, 0, 0.3) nel frame mobile_wx250s/base_link
  // - lunghezza di default 30 cm (se INVIO o input invalido)
  // - tempo tra pubblicazioni di default 2 s (se INVIO o input invalido)
  // - per default pubblica 10 volte la coppia di endpoint (A poi B) con dt tra pubblicazioni
  // - pubblica SOLO su /desired_ee_global_pose

  // 0) Selezione piano
  int plane = 1;
  {
    std::cout << "BACK-AND-FORTH: scegli il piano (INVIO = 1):\n"
                 "  1) Sagittale (parallelo a x-z del drone)\n"
                 "  2) Frontale (parallelo a y-z del drone)\n"
                 "> ";
    std::string line;
    if (std::getline(std::cin, line) && !line.empty()) {
      try {
        plane = std::stoi(line);
      } catch (...) {
        plane = 1;
      }
    }
    if (plane != 2) {
      plane = 1;
    }
  }

  double length_cm = read_double_or_default(
    "BACK-AND-FORTH: lunghezza segmento [cm] (INVIO = 30):\n> ", 30.0);
  if (!(length_cm > 0.0) || !std::isfinite(length_cm)) {
    length_cm = 30.0;
  }
  const double length_m = length_cm / 100.0;

  double dt_s = read_double_or_default(
    "BACK-AND-FORTH: tempo tra due pubblicazioni [s] (INVIO = 2.0):\n> ", 2.0);
  if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
    dt_s = 2.0;
  }

  constexpr int kNumPairs = 10;
  const int num_pubs = 2 * kNumPairs;

  // Endpoint locali nel frame mobile_wx250s/base_link
  const Eigen::Vector3d mid_local = (plane == 2) ? Eigen::Vector3d(0.5, 0.0, 0.3)
                                                : Eigen::Vector3d(0.4, 0.0, 0.3);
  const double half = 0.5 * length_m;
  const Eigen::Vector3d A_local = (plane == 2) ? Eigen::Vector3d(mid_local.x(), mid_local.y() + half, mid_local.z())
                                               : Eigen::Vector3d(mid_local.x() + half, mid_local.y(), mid_local.z());
  const Eigen::Vector3d B_local = (plane == 2) ? Eigen::Vector3d(mid_local.x(), mid_local.y() - half, mid_local.z())
                                               : Eigen::Vector3d(mid_local.x() - half, mid_local.y(), mid_local.z());

  // Attendi pose drone
  rclcpp::Rate wait_rate(10);
  while (rclcpp::ok() && (!has_vehicle_local_position_ || !has_vehicle_attitude_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "In attesa della posa del drone...");
    rclcpp::spin_some(this->get_node_base_interface());
    wait_rate.sleep();
  }
  if (!rclcpp::ok()) {
    return;
  }

  // Trasformazione statica base_link(drone) -> mobile_wx250s/base_link
  pinocchio::forwardKinematics(model_, data_, pinocchio::neutral(model_));
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::FrameIndex arm_base_frame_id = model_.getFrameId("mobile_wx250s/base_link");
  const pinocchio::SE3 &T_base_to_arm_base = data_.oMf[arm_base_frame_id];

  tf2::Transform tf_base_to_arm_base_tf2;
  tf_base_to_arm_base_tf2.setOrigin({T_base_to_arm_base.translation().x(),
                                     T_base_to_arm_base.translation().y(),
                                     T_base_to_arm_base.translation().z()});
  Eigen::Quaterniond q_base(T_base_to_arm_base.rotation());
  q_base.normalize();
  tf2::Quaternion q_tf(q_base.x(), q_base.y(), q_base.z(), q_base.w());
  tf_base_to_arm_base_tf2.setRotation(q_tf);

  // Remark 2: congela la posa di mobile_wx250s/base_link (via posa drone) al momento dell'input.
  // Gli endpoint in world devono restare costanti durante l'esecuzione.
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
  const tf2::Transform tf_world_from_arm_base0 = tf_drone_pose * tf_base_to_arm_base_tf2;

  const tf2::Vector3 pA_world_tf = tf_world_from_arm_base0 * tf2::Vector3(A_local.x(), A_local.y(), A_local.z());
  const tf2::Vector3 pB_world_tf = tf_world_from_arm_base0 * tf2::Vector3(B_local.x(), B_local.y(), B_local.z());

  // Orientazione EE da mantenere costante: usa /ee_world_pose se disponibile, altrimenti FK
  // Remark: l'orientazione agli endpoint deve essere quella all'inizio della traiettoria,
  // cioè dopo che l'utente ha finito di impostare i parametri via CLI (plane/length/dt).
  Eigen::Quaterniond q_world_ee(1.0, 0.0, 0.0, 0.0);
  // prova ad aspettare brevemente la posa EE pubblicata dal controller
  rclcpp::spin_some(this->get_node_base_interface());
  const double ee_wait_timeout_s = 0.5;
  rclcpp::Time ee_wait_t0 = this->now();
  while (rclcpp::ok() && !has_current_ee_pose_ && (this->now() - ee_wait_t0).seconds() < ee_wait_timeout_s) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (has_current_ee_pose_) {
    Eigen::Quaterniond q_start(current_ee_pose_.orientation.w,
                               current_ee_pose_.orientation.x,
                               current_ee_pose_.orientation.y,
                               current_ee_pose_.orientation.z);
    q_start.normalize();
    q_world_ee = q_start;
  } else {
    // Attendi (breve) joint states
    rclcpp::Rate r(50);
    for (int i = 0; i < 50 && rclcpp::ok() && !has_joint_state_; ++i) {
      rclcpp::spin_some(this->get_node_base_interface());
      r.sleep();
    }

    Eigen::VectorXd q = pinocchio::neutral(model_);
    // Base dalla posa del drone
    q[0] = vehicle_local_position_.x;
    q[1] = vehicle_local_position_.y;
    q[2] = vehicle_local_position_.z;
    q[3] = vehicle_attitude_.q[1];
    q[4] = vehicle_attitude_.q[2];
    q[5] = vehicle_attitude_.q[3];
    q[6] = vehicle_attitude_.q[0];
    if (has_joint_state_) {
      for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
        const auto &jname = current_joint_state_.name[i];
        if (!model_.existJointName(jname)) {
          continue;
        }
        const pinocchio::JointIndex jid = model_.getJointId(jname);
        const int idx_q = static_cast<int>(model_.joints[jid].idx_q());
        if (idx_q >= 7 && idx_q < q.size() && i < current_joint_state_.position.size()) {
          q[idx_q] = current_joint_state_.position[i];
        }
      }
    }

    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3 &T_world_ee_now = data_.oMf[ee_frame_id_];
    Eigen::Quaterniond q_fk(T_world_ee_now.rotation());
    q_fk.normalize();
    q_world_ee = q_fk;
  }

  RCLCPP_INFO(this->get_logger(),
              "Back-and-forth: plane=%s, length=%.1f cm, dt=%.2f s, pairs=%d (pub=%d)",
              (plane == 2 ? "frontale(y-z)" : "sagittale(x-z)"), length_cm, dt_s, kNumPairs, num_pubs);
  RCLCPP_INFO(this->get_logger(),
              "Local endpoints (base_link): A=[%.3f %.3f %.3f], B=[%.3f %.3f %.3f]",
              A_local.x(), A_local.y(), A_local.z(), B_local.x(), B_local.y(), B_local.z());
  RCLCPP_INFO(this->get_logger(),
              "Frozen world endpoints: A=[%.3f %.3f %.3f], B=[%.3f %.3f %.3f]",
              pA_world_tf.x(), pA_world_tf.y(), pA_world_tf.z(), pB_world_tf.x(), pB_world_tf.y(), pB_world_tf.z());

  for (int i = 0; i < num_pubs && rclcpp::ok(); ++i) {
    // mantenere attiva l'esecuzione ROS senza aggiornare gli endpoint congelati
    rclcpp::spin_some(this->get_node_base_interface());
    const tf2::Vector3 &p_world_tf = ((i % 2) == 0) ? pA_world_tf : pB_world_tf;

    geometry_msgs::msg::Pose out;
    out.position.x = p_world_tf.x();
    out.position.y = p_world_tf.y();
    out.position.z = p_world_tf.z();
    out.orientation.x = q_world_ee.x();
    out.orientation.y = q_world_ee.y();
    out.orientation.z = q_world_ee.z();
    out.orientation.w = q_world_ee.w();

    desired_ee_global_pose_pub_->publish(out);

    if (i + 1 < num_pubs) {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
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
  // Scelta piano: sagittale (x-z) o frontale (y-z)
  int plane = 1;
  {
    std::cout << "CIRCULAR: scegli il piano (INVIO = 1):\n"
                 "  1) Sagittale (parallelo a x-z del drone)\n"
                 "  2) Frontale (parallelo a y-z del drone)\n"
                 "> ";
    std::string line;
    if (std::getline(std::cin, line) && !line.empty()) {
      try {
        plane = std::stoi(line);
      } catch (...) {
        plane = 1;
      }
    }
    if (plane != 2) {
      plane = 1;
    }
  }

  // Parametri input utente (solo raggio e tempo)
  std::string input;

  std::cout << "Inserire RAGGIO della circonferenza in cm (deve essere < 20 cm):\n> ";
  std::getline(std::cin, input);
  double r_cm = 0.0; try { r_cm = std::stod(input); } catch (...) { r_cm = 0.0; }
  double R = r_cm / 100.0; // metri
  if (R <= 0.0 || R > 0.20) {
    const char* forced = (R <= 0.0 ? "0.10 m" : "0.20 m");
    RCLCPP_WARN(this->get_logger(), "Raggio fuori dai limiti (0 < R < 0.20 m). Imposto R=%s", forced);
    R = (R <= 0.0 ? 0.10 : 0.20);
  }

  std::cout << "Inserire TEMPO DI PERCORRENZA totale in secondi:\n> ";
  std::getline(std::cin, input);
  double T = 0.0; try { T = std::stod(input); } catch (...) { T = 0.0; }
  if (T <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Tempo non valido. Imposto T=4.0 s");
    T = 4.0;
  }

  const int repeats = read_positive_int_or_default(
      "Numero di ripetizioni (INVIO = 1):\n> ", 1);
  const double T_total = T * static_cast<double>(repeats);

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

  // Congela la trasformazione world <- arm_base all'istante di avvio
  tf2::Transform tf_world_from_arm_base0 = tf_drone_pose * tf_base_to_arm_base;
  // Se disponibile, usa la posa reale dell'EE pubblicata dal controller come start (più affidabile di joint_states stale)
  // richiedi qualche aggiornamento ai callback. Aspetta brevemente (timeout) che il controller pubblichi /ee_world_pose
  rclcpp::spin_some(this->get_node_base_interface());
  const double ee_wait_timeout_s = 0.5; // attesa massima per la posa EE pubblicata dal controller
  rclcpp::Time ee_wait_t0 = this->now();
  while (rclcpp::ok() && !has_current_ee_pose_ && (this->now() - ee_wait_t0).seconds() < ee_wait_timeout_s) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  // Attendi (breve) joint states
  rclcpp::Rate r(50);
  for (int i = 0; i < 50 && rclcpp::ok() && !has_joint_state_; ++i) {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
  }
  // Costruisci configurazione q per Pinocchio: base (drone) + giunti manipolatore (se disponibili)
  Eigen::VectorXd q = pinocchio::neutral(model_);
  q[0] = vehicle_local_position_.x;
  q[1] = vehicle_local_position_.y;
  q[2] = vehicle_local_position_.z;
  q[3] = vehicle_attitude_.q[1];
  q[4] = vehicle_attitude_.q[2];
  q[5] = vehicle_attitude_.q[3];
  q[6] = vehicle_attitude_.q[0];
  if (has_joint_state_) {
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
      const auto &jname = current_joint_state_.name[i];
      if (!model_.existJointName(jname)) continue;
      pinocchio::JointIndex jid = model_.getJointId(jname);
      int idx_q = static_cast<int>(model_.joints[jid].idx_q());
      if (idx_q >= 7 && idx_q < q.size()) q[idx_q] = current_joint_state_.position[i];
    }
  }
  geometry_msgs::msg::Pose start_pose_world;
  // Pre-dichiara p_world0 in modo che sia disponibile sia quando la posa EE è fornita
  tf2::Vector3 p_world0;
  if (has_current_ee_pose_) {
    // usa la posa misurata dall'altro nodo (più robusta)
    start_pose_world = current_ee_pose_;
    Eigen::Quaterniond q_start(start_pose_world.orientation.w, start_pose_world.orientation.x,
                               start_pose_world.orientation.y, start_pose_world.orientation.z);
    q_start.normalize();
    q_world_ee = q_start;
    // imposta p_world0 dalla posa misurata
    p_world0 = tf2::Vector3(start_pose_world.position.x, start_pose_world.position.y, start_pose_world.position.z);
  } else {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3 &T_world_ee_now = data_.oMf[ee_frame_id_];
    p_world0 = tf2::Vector3(T_world_ee_now.translation().x(), T_world_ee_now.translation().y(), T_world_ee_now.translation().z());
    start_pose_world.position.x = p_world0.x();
    start_pose_world.position.y = p_world0.y();
    start_pose_world.position.z = p_world0.z();
    Eigen::Quaterniond q_start(T_world_ee_now.rotation()); q_start.normalize();
    start_pose_world.orientation.x = q_start.x();
    start_pose_world.orientation.y = q_start.y();
    start_pose_world.orientation.z = q_start.z();
    start_pose_world.orientation.w = q_start.w();
    q_world_ee = q_start; // orientazione costante
  }
  desired_ee_global_pose_pub_->publish(start_pose_world);
  geometry_msgs::msg::Twist zero_twist; desired_ee_velocity_pub_->publish(zero_twist);
  RCLCPP_INFO(this->get_logger(), "Start EE world: [%.3f %.3f %.3f]", p_world0.x(), p_world0.y(), p_world0.z());

  // assi del drone in world all'istante iniziale
  tf2::Quaternion q_drone0 = tf_drone_pose.getRotation();
  tf2::Vector3 ex_w = tf2::quatRotate(q_drone0, tf2::Vector3(1.0, 0.0, 0.0));
  tf2::Vector3 ey_w = tf2::quatRotate(q_drone0, tf2::Vector3(0.0, 1.0, 0.0));
  tf2::Vector3 ez_w = tf2::quatRotate(q_drone0, tf2::Vector3(0.0, 0.0, 1.0));

  tf2::Vector3 c_world_tf;
  if (plane == 2) {
    // Frontale (y-z): start al punto più basso del cerchio => p_world0 = c - R*ez
    c_world_tf = p_world0 + R * ez_w;
    RCLCPP_INFO(this->get_logger(), "Centro circonferenza (drone y-z) world: [%.3f %.3f %.3f]", c_world_tf.x(), c_world_tf.y(), c_world_tf.z());
  } else {
    // Sagittale (x-z): comportamento esistente
    c_world_tf = p_world0 - R * ex_w;
    RCLCPP_INFO(this->get_logger(), "Centro circonferenza (drone x-z) world: [%.3f %.3f %.3f]", c_world_tf.x(), c_world_tf.y(), c_world_tf.z());
  }

  // Traiettoria con velocità angolare costante intorno all'asse y del drone (fissato a t0)
  const double omega = 2.0 * M_PI / T; // rad/s (1 giro in T)
  rclcpp::Rate rate(100); // 100 Hz
  rclcpp::Time t0_ros = this->now();
  geometry_msgs::msg::Pose pose_msg;
  geometry_msgs::msg::Twist vel_msg;
  while (rclcpp::ok()) {
    rclcpp::Time now_ros = this->now();
    double t = (now_ros - t0_ros).seconds();
    if (t < 0.0) t = 0.0;
    if (t > T_total) t = T_total;

  tf2::Vector3 p_des_tf;
  tf2::Vector3 v_lin_tf;
  if (plane == 2) {
    // Parametrizzazione circolare nel piano y-z del drone a t0.
    // Start al punto più basso: theta0 = -pi/2.
    const double theta = omega * t - (M_PI / 2.0);
    p_des_tf = c_world_tf + (std::cos(theta) * R) * ey_w + (std::sin(theta) * R) * ez_w;
    v_lin_tf = omega * ex_w.cross(p_des_tf - c_world_tf);
  } else {
    // Parametrizzazione circolare nel piano x-z del drone a t0:
    // p_des = c + R*cos(theta)*ex_w + R*sin(theta)*ez_w, con theta=omega*t
    const double theta = omega * t;
    p_des_tf = c_world_tf + (std::cos(theta) * R) * ex_w + (std::sin(theta) * R) * ez_w;
    v_lin_tf = - omega * ey_w.cross(p_des_tf - c_world_tf);
  }

    // Pose world: posizione p_des, orientazione costante q_world_ee
  pose_msg.position.x = p_des_tf.x();
  pose_msg.position.y = p_des_tf.y();
  pose_msg.position.z = p_des_tf.z();
  pose_msg.orientation.x = q_world_ee.x();
  pose_msg.orientation.y = q_world_ee.y();
  pose_msg.orientation.z = q_world_ee.z();
  pose_msg.orientation.w = q_world_ee.w();

    // Velocità desiderata (WORLD): [lin; ang] con ω=0
  vel_msg.linear.x = v_lin_tf.x();
  vel_msg.linear.y = v_lin_tf.y();
  vel_msg.linear.z = v_lin_tf.z();
  vel_msg.angular.x = 0.0;
  vel_msg.angular.y = 0.0;
  vel_msg.angular.z = 0.0;

  desired_ee_global_pose_pub_->publish(pose_msg);
  desired_ee_velocity_pub_->publish(vel_msg);

  if (t >= T_total) break;
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
  std::cout << "POLYLINE: inserisci i waypoints. Ogni riga: \n"
               " - 3 numeri: x y z (posizione)\n"
               " - 7 numeri: x y z qx qy qz qw (posa)\n"
               "Primo punto: premi INVIO per usare la POLYLINE DI DEFAULT (rettangolo) con start dall'EE corrente.\n"
               "Termina l'inserimento con una riga vuota.\n";

  struct WP { Eigen::Vector3d p; bool has_q; Eigen::Quaterniond q; };
  std::vector<WP> wps_local;
  bool use_default_polyline = false;

  // Primo input
  std::cout << "Primo waypoint (o INVIO per default):\n> ";
  std::string first_line;
  if (!std::getline(std::cin, first_line)) return;
  if (first_line.empty()) {
    use_default_polyline = true;
  } else {
    std::stringstream ss(first_line);
    std::vector<double> vals; double x; while (ss >> x) vals.push_back(x);
    if (vals.size() == 3) {
      wps_local.push_back({Eigen::Vector3d(vals[0], vals[1], vals[2]), false, Eigen::Quaterniond(1,0,0,0)});
    } else if (vals.size() == 7) {
      Eigen::Quaterniond q(vals[6], vals[3], vals[4], vals[5]); q.normalize();
      wps_local.push_back({Eigen::Vector3d(vals[0], vals[1], vals[2]), true, q});
    } else {
      std::cout << "Input non valido. Inserire 3 o 7 valori." << std::endl;
    }

    // Eventuali altri waypoint manuali finché non si inserisce riga vuota
    while (rclcpp::ok()) {
      std::cout << "> ";
      std::string line; if (!std::getline(std::cin, line)) break;
      if (line.empty()) break;
      std::stringstream ss2(line);
      std::vector<double> vals2; double x2; while (ss2 >> x2) vals2.push_back(x2);
      if (vals2.size() == 3) {
        wps_local.push_back({Eigen::Vector3d(vals2[0], vals2[1], vals2[2]), false, Eigen::Quaterniond(1,0,0,0)});
      } else if (vals2.size() == 7) {
        Eigen::Quaterniond q2(vals2[6], vals2[3], vals2[4], vals2[5]); q2.normalize();
        wps_local.push_back({Eigen::Vector3d(vals2[0], vals2[1], vals2[2]), true, q2});
      } else {
        std::cout << "Input non valido. Inserire 3 o 7 valori." << std::endl;
      }
    }
  }

  // 2) Tempo massimo per ciascun tratto
  std::cout << "Tempo massimo per ciascun tratto [s] (default 1.0):\n> ";
  std::string input; std::getline(std::cin, input);
  double Tseg = 1.0; try { if (!input.empty()) Tseg = std::stod(input); } catch (...) {}
  if (Tseg <= 0.0) Tseg = 1.0;

  const int repeats = read_positive_int_or_default(
      "Numero di ripetizioni della polyline (INVIO = 1):\n> ", 1);

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

  // 4bis) Se richiesto default: costruisci la polyline di default con start = EE corrente via FK
  Eigen::Quaterniond q_world_ee_current(1,0,0,0);
  bool have_q_world_ee_current = false;
  geometry_msgs::msg::Pose ee_pose_world_fk_at_start;
  bool have_ee_pose_world_fk_at_start = false;
  if (use_default_polyline) {
    // Richiesta: ricava SEMPRE la posa iniziale dell'EE via FK (non usare /ee_world_pose,
    // perché il controller potrebbe non essere avviato).
    // Attendi (breve) joint states; se non arrivano, usa configurazione neutra per FK.
    rclcpp::Rate r(50);
    for (int i = 0; i < 50 && rclcpp::ok() && !has_joint_state_; ++i) {
      rclcpp::spin_some(this->get_node_base_interface());
      r.sleep();
    }

    Eigen::VectorXd q = pinocchio::neutral(model_);
    // Base dalla posa del drone (frozen al momento di avvio della traiettoria)
    q[0] = vehicle_local_position_.x;
    q[1] = vehicle_local_position_.y;
    q[2] = vehicle_local_position_.z;
    q[3] = vehicle_attitude_.q[1];
    q[4] = vehicle_attitude_.q[2];
    q[5] = vehicle_attitude_.q[3];
    q[6] = vehicle_attitude_.q[0];
    if (has_joint_state_) {
      for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
        const auto& jname = current_joint_state_.name[i];
        if (!model_.existJointName(jname)) continue;
        pinocchio::JointIndex jid = model_.getJointId(jname);
        int idx_q = static_cast<int>(model_.joints[jid].idx_q());
        if (idx_q >= 7 && idx_q < q.size()) q[idx_q] = current_joint_state_.position[i];
      }
    }
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3& T_world_ee_now = data_.oMf[ee_frame_id_];

    // Salva posa world dell'EE via FK per usarla come start pubblicato.
    ee_pose_world_fk_at_start.position.x = T_world_ee_now.translation().x();
    ee_pose_world_fk_at_start.position.y = T_world_ee_now.translation().y();
    ee_pose_world_fk_at_start.position.z = T_world_ee_now.translation().z();
    Eigen::Quaterniond q_fk(T_world_ee_now.rotation());
    q_fk.normalize();
    ee_pose_world_fk_at_start.orientation.x = q_fk.x();
    ee_pose_world_fk_at_start.orientation.y = q_fk.y();
    ee_pose_world_fk_at_start.orientation.z = q_fk.z();
    ee_pose_world_fk_at_start.orientation.w = q_fk.w();
    have_ee_pose_world_fk_at_start = true;

    q_world_ee_current = q_fk;
    have_q_world_ee_current = true;

    // Converte start EE world -> locale rispetto a arm_base congelata.
    tf2::Vector3 p_world(T_world_ee_now.translation().x(), T_world_ee_now.translation().y(), T_world_ee_now.translation().z());
    tf2::Vector3 p_local_tf = tf_world_from_arm_base0.inverse() * p_world;
    const Eigen::Vector3d p0_local(p_local_tf.x(), p_local_tf.y(), p_local_tf.z());

    // Costruisci la polyline di default: primo punto = p0_local, poi rettangolo x-z locale
    wps_local.clear();
    wps_local.push_back({p0_local, false, Eigen::Quaterniond(1,0,0,0)});
    wps_local.push_back({Eigen::Vector3d(0.45, 0.0, 0.38), false, Eigen::Quaterniond(1,0,0,0)});
    wps_local.push_back({Eigen::Vector3d(0.45, 0.0, 0.23), false, Eigen::Quaterniond(1,0,0,0)});
    wps_local.push_back({Eigen::Vector3d(0.275, 0.0, 0.23), false, Eigen::Quaterniond(1,0,0,0)});
    wps_local.push_back({Eigen::Vector3d(0.275, 0.0, 0.38), false, Eigen::Quaterniond(1,0,0,0)});
    wps_local.push_back({Eigen::Vector3d(0.45, 0.0, 0.38), false, Eigen::Quaterniond(1,0,0,0)});
  }

  // Verifica numero minimo di waypoints
  if (wps_local.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Servono almeno 2 waypoints per una polyline.");
    return;
  }

  // Espandi la lista dei waypoints per ripetere la traiettoria.
  // Nota: per ripetizioni > 1 viene aggiunto un tratto di rientro dall'ultimo WP al primo WP
  // (linearmente, come gli altri segmenti), poi la sequenza riparte.
  auto same_wp = [](const WP& a, const WP& b) {
    const double pos_eps = 1e-10;
    if ((a.p - b.p).squaredNorm() > pos_eps) return false;
    if (a.has_q != b.has_q) return false;
    if (!a.has_q && !b.has_q) return true;
    // Quaternioni considerati equivalenti anche a segno opposto.
    const double dot = std::abs(a.q.dot(b.q));
    return (1.0 - dot) < 1e-12;
  };
  auto append_wp_if_new = [&](std::vector<WP>& out, const WP& wp) {
    if (out.empty() || !same_wp(out.back(), wp)) out.push_back(wp);
  };
  if (repeats > 1) {
    const std::vector<WP> base = wps_local;
    wps_local.clear();
    wps_local.reserve(base.size() * static_cast<size_t>(repeats) + static_cast<size_t>(repeats));
    for (int k = 0; k < repeats; ++k) {
      if (k == 0) {
        for (const auto& wp : base) append_wp_if_new(wps_local, wp);
      } else {
        // Rientro al primo waypoint e ripartenza: evita duplicati consecutivi
        append_wp_if_new(wps_local, base.front());
        for (size_t j = 1; j < base.size(); ++j) append_wp_if_new(wps_local, base[j]);
      }
    }
  }

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
  // Start: per la polyline di default usa SEMPRE la posa EE via FK.
  if (use_default_polyline && have_ee_pose_world_fk_at_start) {
    first_pose_world = ee_pose_world_fk_at_start;
    q_world_ref = Eigen::Quaterniond(first_pose_world.orientation.w,
                                     first_pose_world.orientation.x,
                                     first_pose_world.orientation.y,
                                     first_pose_world.orientation.z);
    q_world_ref.normalize();
  } else if (has_current_ee_pose_) {
    // Se disponiamo della posa reale dell'EE fornita dal controller, usala come start (misurata)
    first_pose_world = current_ee_pose_;
    q_world_ref = Eigen::Quaterniond(first_pose_world.orientation.w, first_pose_world.orientation.x, first_pose_world.orientation.y, first_pose_world.orientation.z);
  } else if (have_q_world_ee_current) {
    // Se non abbiamo la posa pubblicata, ma abbiamo ricavato l'orientazione via FK (default), usala
    first_pose_world.orientation.x = q_world_ee_current.x();
    first_pose_world.orientation.y = q_world_ee_current.y();
    first_pose_world.orientation.z = q_world_ee_current.z();
    first_pose_world.orientation.w = q_world_ee_current.w();
    q_world_ref = q_world_ee_current;
  } else {
    q_world_ref = Eigen::Quaterniond(first_pose_world.orientation.w, first_pose_world.orientation.x, first_pose_world.orientation.y, first_pose_world.orientation.z);
  }
  desired_ee_global_pose_pub_->publish(first_pose_world);
  geometry_msgs::msg::Twist zero; desired_ee_velocity_pub_->publish(zero);

  rclcpp::Rate rate(100); // 100 Hz
  RCLCPP_INFO(this->get_logger(), "Avvio polyline con %zu waypoints (ripetizioni=%d), T per tratto = %.2f s", wps_local.size(), repeats, Tseg);

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
