#include <open_loop.hpp>

/* Constructor */
OpenLoop::OpenLoop() : Node("open_loop") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  // initialiseServices();
}

/* Destructor */
OpenLoop::~OpenLoop() = default;

/**
 * @brief Load parameters
 */
void OpenLoop::loadParams() {
  surge_enabled_ = declare_parameter<bool>("surge.enabled");
  surge_gain_ = declare_parameter<double>("surge.gain");
}

/**
 * @brief Initialise Subscribers
 */
void OpenLoop::initialiseSubscribers() {
  surge_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
    declare_parameter<std::string>("topics.subscribers.surge_ref"),
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){surgeRefCallback(msg);});
}

/**
 * @brief Initialise Publishers
 */
void OpenLoop::initialisePublishers() {
  thrust_x_pub_ = create_publisher<std_msgs::msg::Float32>(
  declare_parameter<std::string>("topics.publishers.thrust_x"),
  rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void OpenLoop::initialiseServices() {}

void OpenLoop::surgeRefCallback(std_msgs::msg::Float32::SharedPtr msg) {
  /* If open loop for surge is not enabled */
  if (!surge_enabled_) {
    return;
  }

  /* Compute thrust force in surge based on surge reference */
  float32_msg_.data = msg->data * surge_gain_;

  thrust_x_pub_->publish(float32_msg_);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenLoop>());
  rclcpp::shutdown();
  return 0;
}
