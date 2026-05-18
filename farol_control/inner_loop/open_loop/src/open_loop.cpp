#include <open_loop.hpp>

/* Constructor */
OpenLoop::OpenLoop() : Node("open_loop") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  // initialiseServices();
  timer_ = create_timer
  (std::chrono::milliseconds(int(1.0/10.0*1000)), 
  std::bind(&OpenLoop::timerCallback, this));
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
    [this](std_msgs::msg::Float32::SharedPtr msg){ surge_ref_ = msg->data;});
    
  mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
    declare_parameter<std::string>("topics.subscribers.mission_status"),
    rclcpp::QoS(1),
    [this](std_msgs::msg::Int8::SharedPtr msg){mission_status_ = msg->data;});
}

/**
 * @brief Initialise Publishers
 */
void OpenLoop::initialisePublishers() {
  rpm_command_pub_ = create_publisher<control_allocation::msg::ThrusterRPM>(
  declare_parameter<std::string>("topics.publishers.rpm_command"),
  rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void OpenLoop::initialiseServices() {}

void OpenLoop::timerCallback() {
  /* If open loop for surge is not enabled */
  if (!surge_enabled_) return;
  if(mission_status_ == 0){
     rpm_command_msg_.rpm = {};
    rpm_command_msg_.rpm.push_back(0.0);
    rpm_command_msg_.rpm.push_back(0.0);
    rpm_command_pub_->publish(rpm_command_msg_);
    return;
  }
  
  // this is just for magicelectric
  rpm_command_msg_.rpm = {};
  rpm_command_msg_.rpm.push_back(surge_ref_*surge_gain_);
  rpm_command_msg_.rpm.push_back(surge_ref_*surge_gain_);
  rpm_command_pub_->publish(rpm_command_msg_);
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
