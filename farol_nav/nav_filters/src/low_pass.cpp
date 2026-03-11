#include <low_pass.hpp>

// Constructor 
LowPass::LowPass() : Node("low_pass") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseTimers();
}

// Destructor 
LowPass::~LowPass() {
  // Stop the timer 
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void LowPass::loadParams() {
  neglect_current_ = declare_parameter<bool>("neglect_current");
  node_frequency_ = declare_parameter<double>("node_frequency");  
  
  // --- Position --- 
  double omega_cutoff = declare_parameter<double>("position.omega_cutoff");
  int order = declare_parameter<int>("position.order");
  std::string design = declare_parameter<std::string>("position.design");
  std::string method = declare_parameter<std::string>("position.method");
  for (size_t i = 0; i <= 4; ++i)
    lpfs_[i].configure(omega_cutoff, 1/node_frequency_, order, design, method, false);
 
  // --- Orientation --- 
  omega_cutoff = declare_parameter<double>("orientation.omega_cutoff");
  order = declare_parameter<int>("orientation.order");
  design = declare_parameter<std::string>("orientation.design");
  method = declare_parameter<std::string>("orientation.method");
  for (size_t i = 5; i <= 7; ++i)
    lpfs_[i].configure(omega_cutoff, 1/node_frequency_, order, design, method, true);

  // --- Velocity --- 
  omega_cutoff = declare_parameter<double>("velocity.omega_cutoff");
  order = declare_parameter<int>("velocity.order");
  design = declare_parameter<std::string>("velocity.design");
  method = declare_parameter<std::string>("velocity.method");
  for (size_t i = 8; i <= 13; ++i)
    lpfs_[i].configure(omega_cutoff, 1/node_frequency_, order, design, method, false);
  
  // --- Orientation Rate --- 
  omega_cutoff = declare_parameter<double>("orientation_rate.omega_cutoff");
  order = declare_parameter<int>("orientation_rate.order");
  design = declare_parameter<std::string>("orientation_rate.design");
  method = declare_parameter<std::string>("orientation_rate.method");
  for (size_t i = 14; i <=16; ++i)
    lpfs_[i].configure(omega_cutoff, 1/node_frequency_, order, design, method, false);

}

/**
 * @brief Initialise Subscribers
 */
void LowPass::initialiseSubscribers() {
  measurement_sub_ = create_subscription<farol_interfaces::msg::Measurement>(
  declare_parameter<std::string>("topics.subscribers.measurement"),
  rclcpp::QoS(1),
  [this](farol_interfaces::msg::Measurement::SharedPtr msg){measurement_callback(msg);});
  return;
}

/**
 * @brief Initialise Publishers
 */
void LowPass::initialisePublishers() {
  state_pub_ = create_publisher<farol_interfaces::msg::NavigationState>(
  declare_parameter<std::string>("topics.publishers.state"),
  rclcpp::QoS(1));
}

/**
 * @brief Initialise Timers
 */
void LowPass::initialiseTimers() {
  auto period = std::chrono::nanoseconds( static_cast<int64_t>(1e9 / node_frequency_));
  timer_ = create_wall_timer(period, [this]() {timerCallback();});
  return;
}

void LowPass::measurement_callback(farol_interfaces::msg::Measurement::SharedPtr msg) { 
  // Update filter state depending on measurement type 
  switch(msg->type){
    // Orientation: roll, pitch, yaw 
    case farol_interfaces::msg::Measurement::MEAS_ATTITUDE:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION has incorrect length or type.");
        break;
      }
      last_meas_[5] = msg->value[0];
      last_meas_[6] = msg->value[1];
      last_meas_[7] = msg->value[2];
      break;
    // Orientation rate: roll rate, pitch rate, yaw rate 
    case farol_interfaces::msg::Measurement::MEAS_ANGULAR_VELOCITY:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION_RATE has incorrect length or type.");
        break;
      }
      last_meas_[14] = msg->value[0];
      last_meas_[15] = msg->value[1];
      last_meas_[16] = msg->value[2];
      break;
    // UTM position (easting, northing) and UTM zone 
    case farol_interfaces::msg::Measurement::MEAS_UTM_POSITION:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement UTM_POSITION has incorrect length or type.");
        break;
      }
      last_meas_[0] = msg->value[0];
      last_meas_[1] = msg->value[1];
      utm_zone_ = msg->value[2];
      break;
    // Depth 
    case farol_interfaces::msg::Measurement::MEAS_DEPTH:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement DEPTH has incorrect length or type.");
        break;
      }
      last_meas_[2] = msg->value[0];
      break;
    // Altimeter
    case farol_interfaces::msg::Measurement::MEAS_ALTIMETER:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement ALTIMETER has incorrect length or type.");
        break;
      }
      last_meas_[3] = msg->value[0];
      break;
    // Altitude realtive to the ellipsoid, WGS84 
    case farol_interfaces::msg::Measurement::MEAS_ALTITUDE_WGS84:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement ALTITUDE_WGS84 has incorrect length or type.");
        break;
      }
      last_meas_[4] = msg->value[0];
      break;
    // Inertial velocity expressed in the body 
    case farol_interfaces::msg::Measurement::MEAS_INERTIAL_VELOCITY: {
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement BODY_VELOCITY_INERTIAL has incorrect length or type.");
        break;
      }
      last_meas_[8] = msg->value[0];
      last_meas_[9] = msg->value[1];
      last_meas_[10] = msg->value[2];

      // If current is neglected, body velocity relative to the fluid will be the same as inertial one 
      if (neglect_current_) {
        last_meas_[11] = msg->value[0];
        last_meas_[12] = msg->value[1];
        last_meas_[13] = msg->value[2];
      }
      break;}

    // Velocity expressed in the body relative to the fluid 
    case farol_interfaces::msg::Measurement::MEAS_FLUID_VELOCITY:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement BODY_VELOCITY_FLUID has incorrect length or type.");
        break;
      }
      last_meas_[11] = msg->value[0];
      last_meas_[12] = msg->value[1];
      last_meas_[13] = msg->value[2];
      break;
  }
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void LowPass::timerCallback() {
  // run low pass filters
  for (size_t i = 0; i < lpfs_.size(); ++i)
    lpfs_[i].step(last_meas_[i], 1/node_frequency_);
  
  // Fill header 
  filter_state_msg_.header.stamp = clock_.now();
  // fill messages fields with filtered signals
  filter_state_msg_.utm_position.northing = lpfs_[0].y(); 
  filter_state_msg_.utm_position.easting = lpfs_[1].y();
  filter_state_msg_.depth = lpfs_[2].y();
  filter_state_msg_.altimeter = lpfs_[3].y();
  filter_state_msg_.altitude_ellipsoidal = lpfs_[4].y();
  filter_state_msg_.orientation.x = lpfs_[5].y();
  filter_state_msg_.orientation.y = lpfs_[6].y();
  filter_state_msg_.orientation.z = lpfs_[7].y();
  filter_state_msg_.body_velocity_inertial.x = lpfs_[8].y();
  filter_state_msg_.body_velocity_inertial.y = lpfs_[9].y();
  filter_state_msg_.body_velocity_inertial.z = lpfs_[10].y();
  filter_state_msg_.body_velocity_fluid.x = lpfs_[11].y();
  filter_state_msg_.body_velocity_fluid.y = lpfs_[12].y();
  filter_state_msg_.body_velocity_fluid.z = lpfs_[13].y();
  filter_state_msg_.orientation_rate.x = lpfs_[14].y();
  filter_state_msg_.orientation_rate.y = lpfs_[15].y();
  filter_state_msg_.orientation_rate.z = lpfs_[16].y();

  // Compute course angle 
  Eigen::Vector3d v_b(lpfs_[8].y(), lpfs_[9].y(), lpfs_[10].y());
  // Build rotation matrix from body to inertial
  Eigen::Matrix3d R =
    Eigen::Matrix3d(
        Eigen::AngleAxisd(lpfs_[7].y(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(lpfs_[6].y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(lpfs_[5].y(),  Eigen::Vector3d::UnitX())
    );
  // Rotate velocity from body to inertial
  Eigen::Vector3d v_i = R * v_b;
  // Compute course angle
  filter_state_msg_.course_angle = farol_utils::wrapTo2Pi(std::atan2(v_i.y(), v_i.x()));

  // Publish filter state message 
  state_pub_->publish(filter_state_msg_);
}


/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  // initialise ROS2 and start the node 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPass>());
  rclcpp::shutdown();
  return 0;
}
