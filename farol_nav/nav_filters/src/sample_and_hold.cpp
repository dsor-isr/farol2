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
  neglect_current_ = declare_parameter<bool>("neglect_current");
  node_frequency_ = declare_parameter<double>("node_frequency");  
}

/**
 * @brief Initialise Subscribers
 */
void SampleAndHold::initialiseSubscribers() {
  measurement_sub_ = create_subscription<farol_msgs::msg::Measurement>(
    declare_parameter<std::string>("topics.subscribers.measurement"),
    rclcpp::QoS(1),
    [this](farol_msgs::msg::Measurement::SharedPtr msg){measurement_callback(msg);});
  return;
}

/**
 * @brief Initialise Publishers
 */
void SampleAndHold::initialisePublishers() {
  state_pub_ = create_publisher<farol_msgs::msg::NavigationState>(
    declare_parameter<std::string>("topics.publishers.state"),
    rclcpp::QoS(1));
  return;
}

/**
 * @brief Initialise Services
 */
void SampleAndHold::initialiseServices() {
  return;
}

/**
 * @brief Initialise Timers
 */
void SampleAndHold::initialiseTimers() {
  auto period = std::chrono::nanoseconds( static_cast<int64_t>(1e9 / node_frequency_));
  timer_ = create_wall_timer(period, [this]() {timerCallback();});
  return;
}

void SampleAndHold::measurement_callback(farol_msgs::msg::Measurement::ConstSharedPtr msg) {
  // Update filter state depending on measurement type 
  switch(msg->type){
    /* Orientation: roll, pitch, yaw */
    case farol_msgs::msg::Measurement::MEAS_ORIENTATION:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation.x = msg->value[0];
      filter_state_msg_.orientation.y = msg->value[1];
      filter_state_msg_.orientation.z = msg->value[2];
      break;
    /* Orientation rate: roll rate, pitch rate, yaw rate */
    case farol_msgs::msg::Measurement::MEAS_ORIENTATION_RATE:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION_RATE has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation_rate.x = msg->value[0];
      filter_state_msg_.orientation_rate.y = msg->value[1];
      filter_state_msg_.orientation_rate.z = msg->value[2];
      break;
    /* UTM position (easting, northing) and UTM zone */
    case farol_msgs::msg::Measurement::MEAS_UTM_POSITION:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement UTM_POSITION has incorrect length or type.");
        break;
      }
      filter_state_msg_.utm_position.northing = msg->value[0];
      filter_state_msg_.utm_position.easting = msg->value[1];
      filter_state_msg_.utm_position.utm_zone = msg->value[2];
      break;
    /* Depth */
    case farol_msgs::msg::Measurement::MEAS_DEPTH:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement DEPTH has incorrect length or type.");
        break;
      }
      filter_state_msg_.depth = msg->value[0];
      break;
    /* Altimeter */
    case farol_msgs::msg::Measurement::MEAS_ALTIMETER:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement ALTIMETER has incorrect length or type.");
        break;
      }
      filter_state_msg_.altimeter = msg->value[0];
      break;
    /* Altitude realtive to the ellipsoid, WGS84 */
    case farol_msgs::msg::Measurement::MEAS_ALTITUDE_WGS84:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement ALTITUDE_WGS84 has incorrect length or type.");
        break;
      }
      filter_state_msg_.altitude_ellipsoidal = msg->value[0];
      break;
    /* Inertial velocity expressed in the body */
    case farol_msgs::msg::Measurement::MEAS_BODY_VELOCITY_INERTIAL: {
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement BODY_VELOCITY_INERTIAL has incorrect length or type.");
        break;
      }
      filter_state_msg_.body_velocity_inertial.x = msg->value[0];
      filter_state_msg_.body_velocity_inertial.y = msg->value[1];
      filter_state_msg_.body_velocity_inertial.z = msg->value[2];

      /* If current is neglected, body velocity relative to the fluid will be the same as inertial one */
      if (neglect_current_) {
        filter_state_msg_.body_velocity_fluid.x = msg->value[0];
        filter_state_msg_.body_velocity_fluid.y = msg->value[1];
        filter_state_msg_.body_velocity_fluid.z = msg->value[2];
      }

      // Compute course angle 
      Eigen::Vector3d v_b(msg->value[0], msg->value[1], msg->value[2]);
      // Build rotation matrix from body to inertial
      Eigen::Matrix3d R =
        Eigen::Matrix3d(
            Eigen::AngleAxisd(filter_state_msg_.orientation.z,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(filter_state_msg_.orientation.y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(filter_state_msg_.orientation.x,  Eigen::Vector3d::UnitX())
        );
      // Rotate velocity from body to inertial
      Eigen::Vector3d v_i = R * v_b;
      // Compute course angle
      filter_state_msg_.course_angle = farol_utils::wrapTo2Pi(std::atan2(v_i.y(), v_i.x()));
      
      break;}

    /* Velocity expressed in the body relative to the fluid */
    case farol_msgs::msg::Measurement::MEAS_BODY_VELOCITY_FLUID:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement BODY_VELOCITY_FLUID has incorrect length or type.");
        break;
      }
      filter_state_msg_.body_velocity_fluid.x = msg->value[0];
      filter_state_msg_.body_velocity_fluid.y = msg->value[1];
      filter_state_msg_.body_velocity_fluid.z = msg->value[2];
      break;
  }
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */
void SampleAndHold::timerCallback() {
  // Fill header 
  filter_state_msg_.header.stamp = clock_.now();

  // Publish filter state message 
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
