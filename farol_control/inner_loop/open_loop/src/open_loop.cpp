#include <open_loop.hpp>

/* Constructor */
OpenLoop::OpenLoop() : Node("open_loop") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
}

/* Destructor */
OpenLoop::~OpenLoop() {

}

/**
 * @brief Load parameters
 */
void OpenLoop::loadParams() {
  /* Declare parameters */
  declare_parameter<bool>("control.inner_loop.open_loop.surge.enabled", false);
  declare_parameter<double>("control.inner_loop.open_loop.surge.gain", 100.0);

  /* Actually get the parameters */
  surge_enabled_ = get_parameter("control.inner_loop.open_loop.surge.enabled").as_bool();
  surge_gain_ = get_parameter("control.inner_loop.open_loop.surge.gain").as_double();
}

/**
 * @brief Initialise Subscribers
 */
void OpenLoop::initialiseSubscribers() {
  /* Declare parameters */
  declare_parameter<std::string>("control.inner_loop.open_loop.topics.subscribers.surge_ref","dummy");

  surge_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                      get_parameter("control.inner_loop.open_loop.topics.subscribers.surge_ref").as_string(), 
                      1, std::bind(&OpenLoop::surgeRefCallback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Initialise Publishers
 */
void OpenLoop::initialisePublishers() {
  /* Declare parameters */
  declare_parameter<std::string>("control.inner_loop.open_loop.topics.publishers.thrust_x", "dummy");

  thrust_x_pub_ = create_publisher<std_msgs::msg::Float32>(
                    get_parameter("control.inner_loop.open_loop.topics.publishers.thrust_x").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void OpenLoop::initialiseServices() {
  // service servers
  // ...

  // service clients
  // ...

  return;
}

void OpenLoop::surgeRefCallback(const std_msgs::msg::Float32 &msg) {
  /* If open loop for surge is not enabled */
  if (!surge_enabled_) {
    return;
  }

  /* Compute thrust force in surge based on surge reference */
  float32_msg_.data = msg.data * surge_gain_;

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
