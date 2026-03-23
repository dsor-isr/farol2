#include <rudder_control.hpp>

/* Constructor */
RudderControl::RudderControl() : Node("rudder") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  // initialiseServices();
  initialiseTimers();
}

/* Destructor */
RudderControl::~RudderControl() = default;

/**
 * @brief Load parameters
 */
void RudderControl::loadParams() {
  /* Declare parameters */
  node_frequency_ = declare_parameter<double>("node_frequency");
  deadzone_ = declare_parameter<double>("deadzone");
}

/**
 * @brief Initialise Subscribers
 */
void RudderControl::initialiseSubscribers() {  
  rudder_angle_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.subscribers.rudder_angle_ref"),
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){rudder_angle_ref_ = msg->data;});

  rudder_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.subscribers.rudder_angle"),
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){rudder_angle_ = msg->data;});
}

/**
 * @brief Initialise Publishers
 */
void RudderControl::initialisePublishers() {
  rudder_command_pub_ = create_publisher<std_msgs::msg::Float32>(
                          declare_parameter<std::string>("topics.publishers.rudder_command"),
                          rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void RudderControl::initialiseServices() {}

/**
 * @brief Initialise Timers
 */
void RudderControl::initialiseTimers() {
  auto period = std::chrono::nanoseconds( static_cast<int64_t>(1e9 / node_frequency_));
  timer_ = create_timer(period, [this]() {timerCallback();});
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void RudderControl::timerCallback() {

  double rudder_direction = 0;

  // if rudder angle error out of deadzone
  if (abs(rudder_angle_ - rudder_angle_ref_) > deadzone_) { // ~4.0 deg
    if (rudder_angle_ref_ > rudder_angle_) {
      // move rudder to starboard
      rudder_direction = 1.0;
    } else {
      // move rudder to portside
      rudder_direction = -1.0;
    }
  }

  /* Publish rudder command */
  rudder_command_msg_.data = rudder_direction;
  rudder_command_pub_->publish(rudder_command_msg_);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RudderControl>());
  rclcpp::shutdown();
  return 0;
}
