#include <wrench_manager.hpp>

/* Constructor */
WrenchManager::WrenchManager() : Node("wrench_manager") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
WrenchManager::~WrenchManager() = default; 

/**
 * @brief Load parameters
 */
void WrenchManager::loadParams() {
  node_frequency_ = declare_parameter<double>("node_frequency");  
}

/**
 * @brief Initialise Subscribers
 */
void WrenchManager::initialiseSubscribers() {
  // Subscribe to each topic and provide directly a lambda function to save data
  thrust_x_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_THRUST_X,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      wrench_[0] = msg->data;
      last_received_[0] = clock_.now();
    });
  thrust_y_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_THRUST_Y,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      wrench_[1] = msg->data;
      last_received_[1] = clock_.now();
    });
  thrust_z_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_THRUST_Z,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      wrench_[2] = msg->data;
      last_received_[2] = clock_.now();
    });
  torque_x_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_TORQUE_X,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      wrench_[3] = msg->data;
      last_received_[3] = clock_.now();
    });
  torque_y_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_TORQUE_Y,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      wrench_[4] = msg->data;
      last_received_[4] = clock_.now();
    });
  torque_z_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_TORQUE_Z,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      wrench_[5] = msg->data;
      last_received_[5] = clock_.now();
    });
}

/**
 * @brief Initialise Publishers
 */
void WrenchManager::initialisePublishers() {
  body_wrench_request_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
    TOPIC_PUB_BODY_WRENCH_REQUEST,
    rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void WrenchManager::initialiseServices() {return;}

/**
 * @brief Initialise Timers
 */
void WrenchManager::initialiseTimers() {
  auto period = std::chrono::nanoseconds( static_cast<int64_t>(1e9 / node_frequency_));
  timer_ = create_timer(period, [this]() {timerCallback();});
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void WrenchManager::timerCallback() {
  rclcpp::Time now = clock_.now();

  /* Fill header stamp with current time */
  body_wrench_request_msg_.header.stamp = now;

  /* Only fill the body wrench request message if a value has been received for that DOF recently */
  body_wrench_request_msg_.wrench.force.x = ((now - last_received_[0]).seconds() < 2.0/(double)node_frequency_) ? wrench_[0] : 0.0;
  body_wrench_request_msg_.wrench.force.y = ((now - last_received_[1]).seconds() < 2.0/(double)node_frequency_) ? wrench_[1] : 0.0;
  body_wrench_request_msg_.wrench.force.z = ((now - last_received_[2]).seconds() < 2.0/(double)node_frequency_) ? wrench_[2] : 0.0;
  body_wrench_request_msg_.wrench.torque.x = ((now - last_received_[3]).seconds() < 2.0/(double)node_frequency_) ? wrench_[3] : 0.0;
  body_wrench_request_msg_.wrench.torque.y = ((now - last_received_[4]).seconds() < 2.0/(double)node_frequency_) ? wrench_[4] : 0.0;
  body_wrench_request_msg_.wrench.torque.z = ((now - last_received_[5]).seconds() < 2.0/(double)node_frequency_) ? wrench_[5] : 0.0;

  /* Publish message */
  body_wrench_request_pub_->publish(body_wrench_request_msg_);
}


/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchManager>());
  rclcpp::shutdown();
  return 0;
}
