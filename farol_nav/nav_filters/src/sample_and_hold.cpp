#include <sample_and_hold.hpp>

/* Constructor */
SampleAndHold::SampleAndHold() : Node("sample_and_hold") {


  clock_ = this->get_clock();
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseTimers();
}

/* Destructor */
SampleAndHold::~SampleAndHold() {}

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
    rclcpp::QoS(10),
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
  
  debug_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    declare_parameter<std::string>("topics.publishers.course_velings_debug", "dummy"),
    rclcpp::QoS(1));
  debug_pub2_ = create_publisher<std_msgs::msg::Float64>(
    declare_parameter<std::string>("topics.publishers.course_meas_debug", "dummy2"),
    rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void SampleAndHold::initialiseServices() {}

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
    case farol_msgs::msg::Measurement::MEAS_ATTITUDE:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation.x = farol_utils::rad2deg(farol_utils::wrapToPi(msg->value[0]));
      filter_state_msg_.orientation.y = farol_utils::rad2deg(farol_utils::wrapToPi(msg->value[1]));
      filter_state_msg_.orientation.z = farol_utils::rad2deg(farol_utils::wrapTo2Pi(msg->value[2]));
      break;
    /* Orientation rate: roll rate, pitch rate, yaw rate */
    case farol_msgs::msg::Measurement::MEAS_ANGULAR_VELOCITY:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION_RATE has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation_rate.x = farol_utils::rad2deg(msg->value[0]);
      filter_state_msg_.orientation_rate.y = farol_utils::rad2deg(msg->value[1]);
      filter_state_msg_.orientation_rate.z = farol_utils::rad2deg(msg->value[2]);
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
      // Convert UTM position to latitude and longitude
      GeographicLib::UTMUPS::Reverse(
        static_cast<int>(msg->value[2]),
        true,
        msg->value[1],
        msg->value[0],
        filter_state_msg_.global_position.latitude,
        filter_state_msg_.global_position.longitude);

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
    case farol_msgs::msg::Measurement::MEAS_INERTIAL_VELOCITY: {
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement BODY_VELOCITY_INERTIAL has incorrect length or type.");
        break;
      }
      // inertial velocity in the inertial frame (what VN310 gives)
      filter_state_msg_.ned_velocity_inertial.x = msg->value[0];
      filter_state_msg_.ned_velocity_inertial.y = msg->value[1];
      filter_state_msg_.ned_velocity_inertial.z = msg->value[2];

      // Convert inertial velocity from inertial frame to body frame      
      // First, build velocity vector from message  
      Eigen::Vector3d v_i(msg->value[0], msg->value[1], msg->value[2]);
      // Build rotation matrix from body to inertial
      Eigen::Matrix3d R =
        Eigen::Matrix3d(
            Eigen::AngleAxisd(farol_utils::deg2rad(filter_state_msg_.orientation.z),   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(farol_utils::deg2rad(filter_state_msg_.orientation.y), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(farol_utils::deg2rad(filter_state_msg_.orientation.x),  Eigen::Vector3d::UnitX())
        );
      // Rotate velocity from inertial to body
      Eigen::Vector3d v_b = R.transpose() * v_i;
      // Set values
      filter_state_msg_.body_velocity_inertial.x = v_b.x();
      filter_state_msg_.body_velocity_inertial.y = v_b.y();
      filter_state_msg_.body_velocity_inertial.z = v_b.z();

      /* If current is neglected, body velocity relative to the fluid will be the same as inertial one */
      if (neglect_current_) {
        filter_state_msg_.body_velocity_fluid.x = v_b.x();
        filter_state_msg_.body_velocity_fluid.y = v_b.y();
        filter_state_msg_.body_velocity_fluid.z = v_b.z();
        filter_state_msg_.ned_velocity_inertial.x = msg->value[0];
        filter_state_msg_.ned_velocity_inertial.y = msg->value[1];
        filter_state_msg_.ned_velocity_inertial.z = msg->value[2];
      }
      break;}

    /* Velocity expressed in the body relative to the fluid */
    case farol_msgs::msg::Measurement::MEAS_FLUID_VELOCITY:
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
  filter_state_msg_.header.stamp = clock_->now();

  // Complementary filter to estimate course angle
  // Measurement: course angle from UTM position derivative
  double dt = 1.0 / node_frequency_;
  
  // Compute position derivatives
  static double prev_easting = filter_state_msg_.utm_position.easting;
  static double prev_northing = filter_state_msg_.utm_position.northing;
  double vel_easting = (filter_state_msg_.utm_position.easting - prev_easting) / dt;
  double vel_northing = (filter_state_msg_.utm_position.northing - prev_northing) / dt;
  geometry_msgs::msg::Vector3 debug_msg;
  debug_msg.x = vel_northing;
  debug_msg.y = vel_easting;
  debug_msg.z = 0.0;
  debug_pub_->publish(debug_msg);
  prev_easting = filter_state_msg_.utm_position.easting;
  prev_northing = filter_state_msg_.utm_position.northing;
  
  // Measurement: course angle from velocity
  double course_angle_meas = farol_utils::wrapTo2Pi(std::atan2(filter_state_msg_.ned_velocity_inertial.y, filter_state_msg_.ned_velocity_inertial.x));
  debug_pub2_->publish(std_msgs::msg::Float64().set__data(farol_utils::rad2deg(course_angle_meas)));
  
  // Complementary filter: combine gyro integration with measurement
  // course_angle_est = alpha * (gyro_integration) + (1 - alpha) * (measurement)
  double alpha = 0.95; // Gyro weight (adjust as needed)
  
  // Integrate gyro yaw rate
  course_angle_est_ += farol_utils::deg2rad(filter_state_msg_.orientation_rate.z) * dt;
  course_angle_est_ = farol_utils::wrapTo2Pi(course_angle_est_);
  
  // Apply complementary filter
  course_angle_est_ = alpha * course_angle_est_ + (1.0 - alpha) * course_angle_meas;
  course_angle_est_ = farol_utils::wrapTo2Pi(course_angle_est_);
  
  // Compute course angle
  filter_state_msg_.course_angle = farol_utils::rad2deg(course_angle_est_);

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
