#include <rudder.hpp>

/* Constructor */
Rudder::Rudder() : Node("rudder") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
Rudder::~Rudder() {

}

/**
 * @brief Load parameters
 */
void Rudder::loadParams() {
  /* Declare parameters */
  declare_parameter<double>("control.inner_loop.rudder.deadzone", 4.0);

  /* Actually get the parameters */
  deadzone_ = get_parameter("control.inner_loop.rudder.deadzone").as_double() / 180 * M_PI;
}

/**
 * @brief Initialise Subscribers
 */
void Rudder::initialiseSubscribers() {
  /* Declare parameters */
  declare_parameter<std::string>("control.inner_loop.rudder.topics.subscribers.rudder_angle_ref","dummy");
  declare_parameter<std::string>("control.inner_loop.rudder.topics.subscribers.rudder_angle","dummy");

  rudder_angle_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
                            get_parameter("control.inner_loop.rudder.topics.subscribers.rudder_angle_ref").as_string(), 
                            1, std::bind(&Rudder::rudderAngleRefCallback, this, std::placeholders::_1));

  rudder_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
                        get_parameter("control.inner_loop.rudder.topics.subscribers.rudder_angle").as_string(), 
                        1, std::bind(&Rudder::rudderAngleCallback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Initialise Publishers
 */
void Rudder::initialisePublishers() {
  /* Declare parameters */
  declare_parameter<std::string>("control.inner_loop.rudder.topics.publishers.rudder_command", "dummy");

  rudder_command_pub_ = create_publisher<std_msgs::msg::Float32>(
                          get_parameter("control.inner_loop.rudder.topics.publishers.rudder_command").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void Rudder::initialiseServices() {
  // service servers
  // ...

  // service clients
  // ...

  return;
}

/**
 * @brief Initialise Timers
 */
void Rudder::initialiseTimers() {
  /* Declare parameters */
  declare_parameter<int>("control.inner_loop.rudder.node_frequency", 10);

  /* Get node frequency from parameters */
  int freq = get_parameter("control.inner_loop.rudder.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&Rudder::timerCallback, this));
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void Rudder::timerCallback() {
  return;
}

void Rudder::rudderAngleRefCallback(const std_msgs::msg::Float32 &msg) {
  rudder_angle_ref_ = msg.data;

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

void Rudder::rudderAngleCallback(const std_msgs::msg::Float32 &msg) {
  rudder_angle_ = msg.data;
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rudder>());
  rclcpp::shutdown();
  return 0;
}
