#include <sample_and_hold.hpp>

/* Constructor */
SampleAndHold::SampleAndHold() : Node("sample_and_hold") {
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
SampleAndHold::~SampleAndHold() {
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void SampleAndHold::loadParams() {
  /* Declare parameters */

  /* Actually get the parameters */

  return;
}

/**
 * @brief Initialise Subscribers
 */
void SampleAndHold::initialiseSubscribers() {
  /* Declare parameters */
  declare_parameter<std::string>("nav.sample_and_hold.topics.subscribers.measurement","dummy");

  measurement_sub_ = create_subscription<farol_msgs::msg::Measurement>(
                      get_parameter("nav.sample_and_hold.topics.subscribers.measurement").as_string(), 
                      1, std::bind(&SampleAndHold::measurement_callback, this, std::placeholders::_1));

  return;
}

/**
 * @brief Initialise Publishers
 */
void SampleAndHold::initialisePublishers() {
  /* Declare parameters */
  declare_parameter<std::string>("nav.sample_and_hold.topics.publishers.state", "state");

  state_pub_ = create_publisher<farol_msgs::msg::NavigationState>(
                get_parameter("nav.sample_and_hold.topics.publishers.state").as_string(), 1);
}

/**
 * @brief Initialise Services
 */
void SampleAndHold::initialiseServices() {
  // service servers
  // ...

  // service clients
  // ...

  return;
}

/**
 * @brief Initialise Timers
 */
void SampleAndHold::initialiseTimers() {
  /* Get node frequency from parameters */
  declare_parameter<int>("nav.sample_and_hold.node_frequency", 5);
  int freq = get_parameter("nav.sample_and_hold.node_frequency").as_int();

  /* Create timer */
  timer_ = create_wall_timer(std::chrono::milliseconds(int(1.0/freq*1000)), std::bind(&SampleAndHold::timerCallback, this));
}

void SampleAndHold::measurement_callback(const farol_msgs::msg::Measurement &msg) {
  /* Update filter state */
  
  /* Depending on measurement type */
  switch(msg.type){
    /* Orientation: roll, pitch, yaw */
    case msg.MEAS_ORIENTATION:
      if (msg.value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation.x = msg.value[0];
      filter_state_msg_.orientation.y = msg.value[1];
      filter_state_msg_.orientation.z = msg.value[2];
      break;
    /* Orientation rate: roll rate, pitch rate, yaw rate */
    case msg.MEAS_ORIENTATION_RATE:
      if (msg.value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation_rate.x = msg.value[0];
      filter_state_msg_.orientation_rate.y = msg.value[1];
      filter_state_msg_.orientation_rate.z = msg.value[2];
      break;
    /* UTM position (easting, northing) and UTM zone */
    case msg.MEAS_UTM_POSITION:
      if (msg.value.size() != 2 || msg.data.size() == 0) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.utm_position.easting = msg.value[0];
      filter_state_msg_.utm_position.northing = msg.value[1];
      filter_state_msg_.utm_position.utm_zone = msg.data;
      break;
    /* Depth */
    case msg.MEAS_DEPTH:
      if (msg.value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.depth = msg.value[0];
      break;
    /* Altimeter */
    case msg.MEAS_ALTIMETER:
      if (msg.value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.altimeter = msg.value[0];
      break;
    /* Altitude realtive to the ellipsoid, WGS84 */
    case msg.MEAS_ALTITUDE_WGS84:
      if (msg.value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.altitude_ellipsoidal = msg.value[0];
      break;
    /* Inertial velocity expressed in the body */
    case msg.MEAS_BODY_VELOCITY_INERTIAL:
      if (msg.value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.body_velocity_inertial.x = msg.value[0];
      filter_state_msg_.body_velocity_inertial.y = msg.value[1];
      filter_state_msg_.body_velocity_inertial.z = msg.value[2];
      break;
    /* Velocity expressed in the body relative to the fluid */
    case msg.MEAS_BODY_VELOCITY_FLUID:
      if (msg.value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement has incorrect length or type.");
        break;
      }
      filter_state_msg_.body_velocity_fluid.x = msg.value[0];
      filter_state_msg_.body_velocity_fluid.y = msg.value[1];
      filter_state_msg_.body_velocity_fluid.z = msg.value[2];
      break;
  }
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void SampleAndHold::timerCallback() {
  /* Fill header */
  filter_state_msg_.header.stamp = clock_.now();

  /* Publish filter state message */
  state_pub_->publish(filter_state_msg_);
}

/**
 * @brief Main function
 */
int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleAndHold>());
  rclcpp::shutdown();
  return 0;
}
