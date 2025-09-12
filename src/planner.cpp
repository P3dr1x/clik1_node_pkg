#include "clik1_node_pkg/planner.hpp"
#include <string>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"


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
}

void PlannerNode::run() {
  while (rclcpp::ok()) {
    int option = 0;
    while (rclcpp::ok()) {
      std::cout << "What do you want the end-effector to do?" << std::endl;
      std::cout << "1. Positioning" << std::endl;
      std::cout << "> ";
      std::string input; std::getline(std::cin, input);
      try { option = std::stoi(input); } catch (...) { option = 0; }
      if (option == 1) break;
      std::cout << "Opzione non valida. Riprova." << std::endl;
    }
    if (!rclcpp::ok()) break;

    if (option == 1) {
      get_and_transform_desired_pose();
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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  // std::thread spin_thread([&]() { rclcpp::spin(node); });
  node->run();
  rclcpp::shutdown();
  // spin_thread.join();
  return 0;
}
