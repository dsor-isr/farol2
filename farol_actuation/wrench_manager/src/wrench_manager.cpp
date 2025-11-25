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
WrenchManager::~WrenchManager() {

}

/**
 * @brief Load parameters
 */
void WrenchManager::loadParams() {
  return;
}

/**
 * @brief Initialise Subscribers
 */
void WrenchManager::initialiseSubscribers() {
  /* Declare parameters */
  declare_parameter<std::string>("actuation.wrench_manager.topics.subscribers.thrust_x","dummy");
  declare_parameter<std::string>("actuation.wrench_manager.topics.subscribers.thrust_y","dummy");
  declare_parameter<std::string>("actuation.wrench_manager.topics.subscribers.thrust_z","dummy");
  declare_parameter<std::string>("actuation.wrench_manager.topics.subscribers.torque_x","dummy");
  declare_parameter<std::string>("actuation.wrench_manager.topics.subscribers.torque_y","dummy");
  declare_parameter<std::string>("actuation.wrench_manager.topics.subscribers.torque_z","dummy");

  thrust_x_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("actuation.wrench_manager.topics.subscribers.thrust_x").as_string(), 
                    1, std::bind(&WrenchManager::thrustXCallback, this, std::placeholders::_1));

  thrust_y_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("actuation.wrench_manager.topics.subscribers.thrust_y").as_string(), 
                    1, std::bind(&WrenchManager::thrustYCallback, this, std::placeholders::_1));

  thrust_z_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("actuation.wrench_manager.topics.subscribers.thrust_z").as_string(), 
                    1, std::bind(&WrenchManager::thrustZCallback, this, std::placeholders::_1));

  torque_x_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("actuation.wrench_manager.topics.subscribers.torque_x").as_string(), 
                    1, std::bind(&WrenchManager::torqueXCallback, this, std::placeholders::_1));

  torque_y_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("actuation.wrench_manager.topics.subscribers.torque_y").as_string(), 
                    1, std::bind(&WrenchManager::torqueYCallback, this, std::placeholders::_1));

  torque_z_sub_ = create_subscription<std_msgs::msg::Float32>(
                    get_parameter("actuation.wrench_manager.topics.subscribers.torque_z").as_string(), 
                    1, std::bind(&WrenchManager::torqueZCallback, this, std::placeholders::_1));

  

  return;
}

/**
 * @brief Initialise Publishers
 */
void WrenchManager::initialisePublishers() {
  /* Declare parameters */
  declare_parameter<std::string>("actuation.wrench_manager.topics.publishers.body_wrench_request", "dummy");

  body_wrench_request_pub_ = create_publisher<control_allocation::msg::BodyWrenchRequest>(
                              get_parameter("actuation.wrench_manager.topics.publishers.body_wrench_request").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void WrenchManager::initialiseServices() {
  // service servers
  // ...

  // service clients
  // ...

  return;
}

/**
 * @brief Initialise Timers
 */
void WrenchManager::initialiseTimers() {
  /* Declare parameters */
  declare_parameter<int>("actuation.wrench_manager.node_frequency", 10);

  /* Get node frequency from parameters */
  freq_ = get_parameter("actuation.wrench_manager.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq_*1000)), std::bind(&WrenchManager::timerCallback, this));
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
  body_wrench_request_msg_.wrench.force.x = ((now - last_received_[0]).seconds() < 2.0/(double)freq_) ? wrench_[0] : 0.0;
  body_wrench_request_msg_.wrench.force.y = ((now - last_received_[1]).seconds() < 2.0/(double)freq_) ? wrench_[1] : 0.0;
  body_wrench_request_msg_.wrench.force.z = ((now - last_received_[2]).seconds() < 2.0/(double)freq_) ? wrench_[2] : 0.0;
  body_wrench_request_msg_.wrench.torque.x = ((now - last_received_[3]).seconds() < 2.0/(double)freq_) ? wrench_[3] : 0.0;
  body_wrench_request_msg_.wrench.torque.y = ((now - last_received_[4]).seconds() < 2.0/(double)freq_) ? wrench_[4] : 0.0;
  body_wrench_request_msg_.wrench.torque.z = ((now - last_received_[5]).seconds() < 2.0/(double)freq_) ? wrench_[5] : 0.0;

  /* Publish message */
  body_wrench_request_pub_->publish(body_wrench_request_msg_);
}

void WrenchManager::thrustXCallback(const std_msgs::msg::Float32 &msg) {
  wrench_[0] = msg.data;
  last_received_[0] = clock_.now();
}

void WrenchManager::thrustYCallback(const std_msgs::msg::Float32 &msg) {
  wrench_[1] = msg.data;
  last_received_[1] = clock_.now();
}

void WrenchManager::thrustZCallback(const std_msgs::msg::Float32 &msg) {
  wrench_[2] = msg.data;
  last_received_[2] = clock_.now();
}

void WrenchManager::torqueXCallback(const std_msgs::msg::Float32 &msg) {
  wrench_[3] = msg.data;
  last_received_[3] = clock_.now();
}

void WrenchManager::torqueYCallback(const std_msgs::msg::Float32 &msg) {
  wrench_[4] = msg.data;
  last_received_[4] = clock_.now();
}

void WrenchManager::torqueZCallback(const std_msgs::msg::Float32 &msg) {
  wrench_[5] = msg.data;
  last_received_[5] = clock_.now();
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
