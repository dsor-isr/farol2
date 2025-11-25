#include <throttle_conversion.hpp>

/* Constructor */
ThrottleConversion::ThrottleConversion() : Node("throttle_conversion", 
                                      rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
}

/* Destructor */
ThrottleConversion::~ThrottleConversion() {
  
}

/**
 * @brief Initialise Subscribers
 */
void ThrottleConversion::initialiseSubscribers() {
  rpm_command_sub_ = create_subscription<control_allocation::msg::ThrusterRPM>(
                      get_parameter("actuation.throttle_conversion.topics.subscribers.rpm_command").as_string(), 
                      1, std::bind(&ThrottleConversion::rpmCommandCallback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Load parameters
 * Thruster configuration parameters are loaded under some assumptions.
 * In the future, if ROS2 enables native parameter loading for dicts and 
 * other complex types, this method should be adapted for further robustness.
 */
void ThrottleConversion::loadParams() {
  k_ = get_parameter("actuation.thrusters.throttle_conversion.k").as_double();
}

/**
 * @brief Initialise Publishers
 */
void ThrottleConversion::initialisePublishers() {
  throttle_command_pub_ = create_publisher<farol_msgs::msg::Thruster>(
                            get_parameter("actuation.throttle_conversion.topics.publishers.throttle_command").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void ThrottleConversion::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Compute throttle for each thruster based on rpm for that thruster.
 */
void ThrottleConversion::rpmCommandCallback(const control_allocation::msg::ThrusterRPM &msg) {
  throttle_command_msg_.header.stamp = clock_.now();
  throttle_command_msg_.value = {};
  
  for (int i = 0; i < (int)msg.rpm.size(); i++) {
    throttle_command_msg_.value.push_back(msg.rpm[0]/k_);
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
