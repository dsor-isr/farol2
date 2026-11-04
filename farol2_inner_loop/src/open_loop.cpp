#include <open_loop.hpp>

#include <algorithm>
#include <limits>

/* Constructor */
OpenLoop::OpenLoop() : Node("open_loop") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  // initialiseServices();
  timer_ = create_timer
  (std::chrono::milliseconds(static_cast<int>(1000.0 / node_frequency_)),
  std::bind(&OpenLoop::timerCallback, this));
}

/* Destructor */
OpenLoop::~OpenLoop() = default;

/**
 * @brief Load parameters
 */
void OpenLoop::loadParams() {
  surge_enabled_ = declare_parameter<bool>("surge.enabled", false);
  surge_gain_ = declare_parameter<double>("surge.gain", 1.0);
  // Saturation of the commanded rpm (defaults: no saturation). Set these to the limits
  // of rpm_conversion in allocation.yaml, this node bypasses the allocation node.
  rpm_max_ = declare_parameter<double>("surge.rpm_max", std::numeric_limits<double>::infinity());
  rpm_min_ = declare_parameter<double>("surge.rpm_min", -std::numeric_limits<double>::infinity());
  num_thrusters_ = declare_parameter<int>("surge.num_thrusters", 2);
  node_frequency_ = declare_parameter<double>("node_frequency", 10.0);
  ref_timeout_ = declare_parameter<double>("ref_timeout", 0.2);
  if (node_frequency_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "node_frequency must be positive, using 10 Hz");
    node_frequency_ = 10.0;
  }
  if (num_thrusters_ < 1) {
    num_thrusters_ = 1;
  }
}



/**
 * @brief Initialise Subscribers
 */
void OpenLoop::initialiseSubscribers() {
  surge_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
    TOPIC_SUB_SURGE_REF,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg){
      surge_ref_ = msg->data;
      last_surge_ref_time_ = this->now();
      has_surge_ref_ = true;
    });
    
  mission_status_sub_ = create_subscription<std_msgs::msg::Int8>(
    TOPIC_SUB_MISSION_STATUS,
    rclcpp::QoS(1),
    [this](std_msgs::msg::Int8::SharedPtr msg){mission_status_ = msg->data;});
}

/**
 * @brief Initialise Publishers
 */
void OpenLoop::initialisePublishers() {
  rpm_command_pub_ = create_publisher<farol2_interfaces::msg::ThrusterRPM>(
  TOPIC_PUB_RPM_COMMAND,
  rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void OpenLoop::initialiseServices() {}

void OpenLoop::timerCallback() {
  /* If open loop for surge is not enabled */
  if (!surge_enabled_) return;

  if (!has_surge_ref_ || (this->now() - last_surge_ref_time_).seconds() > ref_timeout_) {
    return;
  }


  if(mission_status_ == 0){
    rpm_command_msg_.rpm.assign(num_thrusters_, 0.0);
    rpm_command_pub_->publish(rpm_command_msg_);
    return;
  }
  
  // Same rpm for all thrusters (common mode), saturated to [rpm_min, rpm_max]
  const double rpm = std::clamp(surge_ref_ * surge_gain_, rpm_min_, rpm_max_);
  rpm_command_msg_.rpm.assign(num_thrusters_, rpm);
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