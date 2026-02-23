#include <throttle_conversion.hpp>

/* Constructor */
ThrottleConversion::ThrottleConversion() : Node("throttle_conversion") {
  loadParams();
  initialisePublishers();
  initialiseSubscribers();
  // initialiseServices();
}

/* Destructor */
ThrottleConversion::~ThrottleConversion() = default;

/**
 * @brief Initialise Subscribers
 */
void ThrottleConversion::initialiseSubscribers() {
  rpm_command_sub_ = create_subscription<control_allocation::msg::ThrusterRPM>(
  declare_parameter<std::string>("topics.subscribers.rpm_command"),
  rclcpp::QoS(1),
  [this](control_allocation::msg::ThrusterRPM::SharedPtr msg){rpmCommandCallback(msg);});
}

/**
 * @brief Load parameters
 */
void ThrottleConversion::loadParams() {
  k_ = declare_parameter<double>("k");
}

/**
 * @brief Initialise Publishers
 */
void ThrottleConversion::initialisePublishers() {
  throttle_command_pub_ = create_publisher<farol_msgs::msg::Thruster>(
  declare_parameter<std::string>("topics.publishers.throttle_command"),
  rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void ThrottleConversion::initialiseServices() {}

/**
 * @brief Compute throttle for each thruster based on rpm for that thruster.
 */
void ThrottleConversion::rpmCommandCallback(control_allocation::msg::ThrusterRPM::SharedPtr msg) {
  throttle_command_msg_.header.stamp = clock_.now();
  throttle_command_msg_.value = {};
  
  for (int i = 0; i < (int)msg->rpm.size(); i++) {
    throttle_command_msg_.value.push_back(msg->rpm[0]/k_);
  }

  /* Publish */
  throttle_command_pub_->publish(throttle_command_msg_);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrottleConversion>());
  rclcpp::shutdown();
  return 0;
}
