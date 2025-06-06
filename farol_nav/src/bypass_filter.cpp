#include <bypass_filter.hpp>

/* Constructor */
BypassFilter::BypassFilter() : Node("bypass_filter") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
BypassFilter::~BypassFilter() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void BypassFilter::loadParams() {
  /* Declare all parameters */
  declare_parameter<int>("nav.bypass_filter.node_frequency", 5);
  declare_parameter<std::string>("nav.bypass_filter.topics.subscribers.position", "test");
  declare_parameter<int>("nav.bypass_filter.test", 0);

  /* Actually get the parameters */
  int freq = get_parameter("nav.bypass_filter.node_frequency").as_int();
  std::string topic = get_parameter("nav.bypass_filter.topics.subscribers.position").as_string();
  int nav = get_parameter("nav.bypass_filter.test").as_int();

  RCLCPP_INFO(get_logger(), "Param1: '%d'", freq);
  RCLCPP_INFO(get_logger(), "Param2: '%s'", topic.c_str());
  RCLCPP_INFO(get_logger(), "Param3: '%d'", nav);

  return;
}

/**
 * @brief Initialise Subscribers
 */
void BypassFilter::initialiseSubscribers() {
  return;
}

/**
 * @brief Initialise Publishers
 */
void BypassFilter::initialisePublishers() {
  /* Declare parameters */
  declare_parameter<std::string>("nav.bypass_filter.topics.publishers.state", "state");

  state_pub_ = create_publisher<farol_msgs::msg::NavigationState>(
                get_parameter("nav.bypass_filter.topics.publishers.state").as_string(), 10);
}

/**
 * @brief Initialise Services
 */
void BypassFilter::initialiseServices() {
  // service servers
  // ...

  // service clients
  // ...

  return;
}

/**
 * @brief Initialise Timers
 */
void BypassFilter::initialiseTimers() {
  timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&BypassFilter::timerCallback, this));
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void BypassFilter::timerCallback() {
  farol_msgs::msg::NavigationState msg;
  rclcpp::Clock clock;

  msg.header.stamp = clock.now();
  msg.depth = 100;
  
  state_pub_->publish(msg);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BypassFilter>());
  rclcpp::shutdown();
  return 0;
}
