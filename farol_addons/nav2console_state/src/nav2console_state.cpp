#include <nav2console_state.hpp>

/* Constructor */
Nav2ConsoleState::Nav2ConsoleState() : Node("nav2console_state", 
                                            rclcpp::NodeOptions()
                                              .allow_undeclared_parameters(true)
                                              .automatically_declare_parameters_from_overrides(true)) {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
}

/* Destructor */
Nav2ConsoleState::~Nav2ConsoleState() {
  return;
}

/**
 * @brief Load parameters
 */
void Nav2ConsoleState::loadParams() {
  return;
}

/**
 * @brief Initialise Subscribers
 */
void Nav2ConsoleState::initialiseSubscribers() {
  nav_state_sub_ = create_subscription<farol_msgs::msg::NavigationState>(
                get_parameter("addons.nav2console_state.topics.subscribers.nav_state").as_string(), 
                1, std::bind(&Nav2ConsoleState::nav_state_callback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Initialise Publishers
 */
void Nav2ConsoleState::initialisePublishers() {
  console_state_pub_ = create_publisher<farol_msgs::msg::StateConsole>(
                        get_parameter("addons.nav2console_state.topics.publishers.console_state").as_string(), 1);
}

void Nav2ConsoleState::nav_state_callback(const farol_msgs::msg::NavigationState &msg) {
  /* Update console_state_msg_ */
  console_state_msg_.header.stamp = clock_.now();

  /* PLACEHOLDER: these values should be actually included in the nav state  */
  /*              (via a measurement from drivers) and updated here then     */
  console_state_msg_.gps_good = 3;
  console_state_msg_.imu_good = 3;

  /* Depth */
  console_state_msg_.depth = msg.depth;
  
  /* XYZ */
  console_state_msg_.x = msg.utm_position.easting;
  console_state_msg_.y = msg.utm_position.northing;
  console_state_msg_.z = msg.depth;

  /* Vx / Vy / Vz */
  /* PLACEHOLDER: should be updated with actual velocities in the inertial frame */
  console_state_msg_.vx = -1.0;
  console_state_msg_.vy = -1.0;
  console_state_msg_.vz = -1.0;

  /* Surge */
  console_state_msg_.surge = msg.body_velocity_fluid.x;

  /* Orientation */
  console_state_msg_.yaw = msg.orientation.z / M_PI * 180;
  console_state_msg_.pitch = msg.orientation.y / M_PI * 180;
  console_state_msg_.roll = msg.orientation.x / M_PI * 180;

  /* Orientation Rate */
  console_state_msg_.yaw_rate = msg.orientation_rate.z / M_PI * 180;
  console_state_msg_.pitch_rate = msg.orientation_rate.y / M_PI * 180;
  console_state_msg_.roll_rate = msg.orientation_rate.x / M_PI * 180;

  /* Inside Pressure, Battery Level, Altitude */
  /* PLACEHOLDER: should be should be actually included in the nav state  */
  /*              (via a measurement from drivers) and updated here then */
  console_state_msg_.in_pressure = -1.0;
  console_state_msg_.in_pressure_dot = -1.0;
  console_state_msg_.battery_level = 0;
  console_state_msg_.altitude = -1.0;

  /* Maybe also add some safeguards and conditions around this flag... */
  console_state_msg_.status = farol_msgs::msg::StateConsole::STATUS_ALL_OK;

  console_state_pub_->publish(console_state_msg_);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2ConsoleState>());
  rclcpp::shutdown();
  return 0;
}
