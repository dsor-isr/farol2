#include <navigation_filter_node.hpp>

/* Constructor */
NavigationFilterNode::NavigationFilterNode() : Node("navigation_filter"), count_(0) {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
NavigationFilterNode::~NavigationFilterNode() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void NavigationFilterNode::loadParams() {
  return;
}

/**
 * @brief Initialise Subscribers
 */
void NavigationFilterNode::initialiseSubscribers() {
  return;
}

/**
 * @brief Initialise Publishers
 */
void NavigationFilterNode::initialisePublishers() {

  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
}

/**
 * @brief Initialise Services
 */
void NavigationFilterNode::initialiseServices() {
  // service servers
  // ...

  // service clients
  // ...

  return;
}

/**
 * @brief Initialise Timers
 */
void NavigationFilterNode::initialiseTimers() {
  timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavigationFilterNode::timerCallback, this));
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void NavigationFilterNode::timerCallback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  
  RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
  
  publisher_->publish(message);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationFilterNode>());
  rclcpp::shutdown();
  return 0;
}
