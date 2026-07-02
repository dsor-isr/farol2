#include <magic_electric_sim.hpp>

/* Constructor */
MagicElectricSim::MagicElectricSim() : Node("magic_electric_sim")
{
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
MagicElectricSim::~MagicElectricSim()
{
  /* Stop the timer */
  timer_->cancel();
}

/**
 * @brief Load parameters
 */
void MagicElectricSim::loadParams()
{

  //////////////////////////////////////////////////////////////////////////////////////

  node_frequency_ = declare_parameter<int>("node_frequency");
  node_period_ = (1.0 / static_cast<double>(node_frequency_));
  frame_prefix_ = declare_parameter<std::string>("frame_prefix", "");

  // ==========================
  // INITIAL STATE
  // ==========================

  auto vel_param = declare_parameter<std::vector<double>>("initial_state.body_velocity");
  body_velocity_[0] = vel_param[0];
  body_velocity_[1] = vel_param[1];
  body_velocity_[2] = vel_param[2];

  auto orient_param = declare_parameter<std::vector<double>>("initial_state.orientation");
  orientation_[0] = orient_param[0];
  orientation_[1] = orient_param[1];
  orientation_[2] = orient_param[2];

  auto orient_rate_param = declare_parameter<std::vector<double>>("initial_state.orientation_rate");
  orientation_rate_[0] = orient_rate_param[0];
  orientation_rate_[1] = orient_rate_param[1];
  orientation_rate_[2] = orient_rate_param[2];

  body_acceleration_[0] = 0.0;
  body_acceleration_[1] = 0.0;
  body_acceleration_[2] = 0.0;

  angular_acceleration_[0] = 0.0;
  angular_acceleration_[1] = 0.0;
  angular_acceleration_[2] = 0.0;

  position_dot_[0] = 0.0;
  position_dot_[1] = 0.0;
  position_dot_[2] = 0.0;

  rudder_velocity_[0] = 0.0;
  rudder_velocity_[1] = 0.0;
  rudder_velocity_[2] = 0.0;

  // ==========================
  // ENVIRONMENT
  // ==========================

  fluid_density = declare_parameter<double>("environment.fluid_density");
  auto current_vec = declare_parameter<std::vector<double>>("environment.current");
  curr_vel_ = current_vec[0];
  curr_dir_ = current_vec[1];

  // ==========================
  // VEHICLE PARAMETERS
  // ==========================

  mu = declare_parameter<double>("vehicle.mass");
  zg = declare_parameter<double>("vehicle.zg");
  m_u = declare_parameter<double>("vehicle.m_u");
  X_u = declare_parameter<double>("vehicle.X_u");
  X_uu = declare_parameter<double>("vehicle.X_uu");
  m_uv = declare_parameter<double>("vehicle.m_uv");
  m_v = m_u - m_uv;
  N_r = declare_parameter<double>("vehicle.N_r");
  N_rr = declare_parameter<double>("vehicle.N_rr");
  K_L = declare_parameter<double>("vehicle.K_L");
  K_D0 = declare_parameter<double>("vehicle.K_D0");
  K_D1 = declare_parameter<double>("vehicle.K_D1");
  m_r = declare_parameter<double>("vehicle.m_r");
  Y_v = declare_parameter<double>("vehicle.Y_v");
  Y_vv = declare_parameter<double>("vehicle.Y_vv");

  // ==========================
  // RUDDER
  // ==========================

  rudder_actuation_sim_ = declare_parameter<bool>("vehicle.allocation.rudder.sim_rudder");
  max_rudder_angle_ = declare_parameter<double>("vehicle.allocation.rudder.max_angle_deg");
  min_rudder_angle_ = declare_parameter<double>("vehicle.allocation.rudder.min_angle_deg");
  max_rudder_angle_ = deg_to_rad(max_rudder_angle_);
  min_rudder_angle_ = deg_to_rad(min_rudder_angle_);
  rudder_ang_vel_ = deg_to_rad(declare_parameter<double>("vehicle.allocation.rudder.ang_vel"));

  // ==========================
  // PROPULSION
  // ==========================

  fixed_rpm_ = declare_parameter<bool>("vehicle.allocation.propulsion.fixed_rpm");
  fixed_rpm_value_ = declare_parameter<double>("vehicle.allocation.propulsion.fixed_rpm_value");
  max_rpm_ = declare_parameter<double>("vehicle.allocation.propulsion.max_rpm");
  min_rpm_ = declare_parameter<double>("vehicle.allocation.propulsion.min_rpm");
  K_T_BP = declare_parameter<double>("vehicle.allocation.propulsion.K_T_BP");
  prop_pitch = declare_parameter<double>("vehicle.allocation.propulsion.prop_pitch");
  D = declare_parameter<double>("vehicle.allocation.propulsion.D");
  l_ = declare_parameter<double>("vehicle.allocation.propulsion.l_");
  deadzone_propeller_pos_ = declare_parameter<double>("vehicle.allocation.propulsion.deadzone_pos");
  deadzone_propeller_neg_ = declare_parameter<double>("vehicle.allocation.propulsion.deadzone_neg");
  rpm_[0] = 0.0;

  // Sensor params
  gnss_activate_         = declare_parameter<bool>("sensor.gnss");
  gnss_velocity_over_ground_activate_ =
    declare_parameter<bool>("sensor.gnss_velocity_over_ground");
  depth_sensor_activate_ = declare_parameter<bool>("sensor.depth_sensor");
  imu_activate_          = declare_parameter<bool>("sensor.imu");
  noise_activate_        = declare_parameter<bool>("sensor.noise.activate");

  auto p_bias = declare_parameter<std::vector<double>>("sensor.noise.position.bias");
  auto p_var  = declare_parameter<std::vector<double>>("sensor.noise.position.variance");
  auto o_bias = declare_parameter<std::vector<double>>("sensor.noise.orientation.bias");
  auto o_var  = declare_parameter<std::vector<double>>("sensor.noise.orientation.variance");
  auto v_bias = declare_parameter<std::vector<double>>("sensor.noise.body_velocity.bias");
  auto v_var  = declare_parameter<std::vector<double>>("sensor.noise.body_velocity.variance");
  auto r_bias = declare_parameter<std::vector<double>>("sensor.noise.orientation_rate.bias");
  auto r_var  = declare_parameter<std::vector<double>>("sensor.noise.orientation_rate.variance");
  for (int i = 0; i < 3; ++i) {
    pos_bias[i] = p_bias[i]; pos_variance[i] = p_var[i];
    ori_bias[i] = o_bias[i]; ori_variance[i] = o_var[i];
    vel_bias[i] = v_bias[i]; vel_variance[i] = v_var[i];
    ori_rate_bias[i] = r_bias[i]; ori_rate_variance[i] = r_var[i];
  }

  auto pos_param = declare_parameter<std::vector<double>>("initial_state.position");
  origin_latitude_  = pos_param[0];
  origin_longitude_ = pos_param[1];
  position_[2] = pos_param[2];
  GeographicLib::UTMUPS::Forward(origin_latitude_, origin_longitude_, utm_zone_, northp_, easting_, northing_);
}

/**
 * @brief Initialise Subscribers
 */
void MagicElectricSim::initialiseSubscribers()
{

  rpm_sub_ = create_subscription<farol2_interfaces::msg::ThrusterRPM>(
      TOPIC_SUB_RPM_COMMAND,
      1, std::bind(&MagicElectricSim::rpmCallback, this, std::placeholders::_1));

  if (rudder_actuation_sim_)
  {
    rudder_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
        TOPIC_SUB_RUDDER_CMD,
        1, std::bind(&MagicElectricSim::rudderAngleCallback, this, std::placeholders::_1));
  }
  else
  {
    rudder_ref_sub_ = create_subscription<std_msgs::msg::Float32>(
        TOPIC_SUB_RUDDER_REF,
        1, std::bind(&MagicElectricSim::rudderAngleCallback, this, std::placeholders::_1));
  }

  return;
}

/**
 * @brief Initialise Publishers
 */
void MagicElectricSim::initialisePublishers()
{
  position_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_POSITION, 1);
  velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_BODY_VELOCITY, 1);
  orientation_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_ORIENTATION, 1);
  oreintation_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_ORIENTATION_RATE, 1);
  body_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_BODY_ACCELERATION, 1);
  angular_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_ANGULAR_ACCELERATION, 1);
  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      TOPIC_PUB_JOINT_STATES, 1);

  if (rudder_actuation_sim_)
  {
    control_surface_deflection_pub_ =
      create_publisher<farol2_interfaces::msg::ControlSurfaceDeflection>(
        TOPIC_PUB_CONTROL_SURFACE_DEFLECTION, 1);
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      TOPIC_PUB_IMU, 1);
  gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      TOPIC_PUB_GNSS, 1);
  utm_ned_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      TOPIC_PUB_UTM_NED, 1);
  velocity_over_ground_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      TOPIC_PUB_VELOCITY_OVER_GROUND, 1);
  velocity_through_water_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      TOPIC_PUB_VELOCITY_THROUGH_WATER, 1);
  depth_pub_ = create_publisher<farol2_interfaces::msg::Depth>(
      TOPIC_PUB_DEPTH, 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return;
}

/**
 * @brief Initialise Services
 */
void MagicElectricSim::initialiseServices()
{
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void MagicElectricSim::initialiseTimers()
{

  /* Create timer */
  timer_ = create_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(std::llround(node_period_ * 1e9))),
      std::bind(&MagicElectricSim::timerCallback, this));
}

/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */

void MagicElectricSim::rudderAngleCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!rudder_actuation_sim_)
  {
    rudder_angle_ = deg_to_rad(msg->data);
    rudder_angle_ = std::clamp(rudder_angle_, min_rudder_angle_, max_rudder_angle_);
    return;
  }

  rudder_command_ = std::clamp(static_cast<double>(msg->data), -1.0, 1.0);
}

void MagicElectricSim::rpmCallback(const farol2_interfaces::msg::ThrusterRPM::SharedPtr msg)
{

  rpm_[0] = msg->rpm[0];

  rpm_[0] = std::max(std::min(rpm_[0], max_rpm_), min_rpm_);
}

void MagicElectricSim::timerCallback()
{
  if (rudder_actuation_sim_)
  {
    updateRudder(rudder_command_, node_period_);
  }
  const auto stamp = get_clock()->now();

  updateState();

  geometry_msgs::msg::Vector3 pos_msg, body_vel_msg, ori_msg, ori_rate_msg, body_acc_msg, ang_acc_msg;

  pos_msg.x = position_[0];
  pos_msg.y = position_[1];
  pos_msg.z = position_[2];

  body_vel_msg.x = body_velocity_[0];
  body_vel_msg.y = body_velocity_[1];
  body_vel_msg.z = body_velocity_[2];

  ori_msg.x = rad_to_deg(orientation_[0]);
  ori_msg.y = rad_to_deg(orientation_[1]);
  ori_msg.z = rad_to_deg(orientation_[2]);

  ori_rate_msg.x = rad_to_deg(orientation_rate_[0]);
  ori_rate_msg.y = rad_to_deg(orientation_rate_[1]);
  ori_rate_msg.z = rad_to_deg(orientation_rate_[2]);

  body_acc_msg.x = body_acceleration_[0];
  body_acc_msg.y = body_acceleration_[1];
  body_acc_msg.z = body_acceleration_[2];

  ang_acc_msg.x = angular_acceleration_[0];
  ang_acc_msg.y = angular_acceleration_[1];
  ang_acc_msg.z = angular_acceleration_[2];

  position_pub_->publish(pos_msg);
  velocity_pub_->publish(body_vel_msg);
  orientation_pub_->publish(ori_msg);
  oreintation_rate_pub_->publish(ori_rate_msg);
  body_acceleration_pub_->publish(body_acc_msg);
  angular_acceleration_pub_->publish(ang_acc_msg);

  if (rudder_actuation_sim_)
  {
    farol2_interfaces::msg::ControlSurfaceDeflection control_surface_msg;
    control_surface_msg.header.stamp = stamp;
    control_surface_msg.header.frame_id = frame_prefix_ + "rudder_link";
    control_surface_msg.deflection_angle.push_back(rudder_angle_ * 180.0 / M_PI);
    control_surface_deflection_pub_->publish(control_surface_msg);
  }

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = stamp;
  joint_state_msg.name.push_back("base_to_rudder_middle");
  joint_state_msg.position.push_back(rudder_angle_);
  joint_states_pub_->publish(joint_state_msg);

  publishWorldTransform(stamp);
  publishMeasurements();

  return;
}

void MagicElectricSim::updateRudder(double command, double dt)
{

  if (command > 0.5)
  {
    rudder_angle_ += rudder_ang_vel_ * dt;
  }
  else if (command < -0.5)
  {
    rudder_angle_ -= rudder_ang_vel_ * dt;
  }

  rudder_angle_ = std::clamp(rudder_angle_, min_rudder_angle_, max_rudder_angle_);
}

void MagicElectricSim::updateState()
{

  // DYNAMICS

  if (fixed_rpm_)
  {
    rpm_[0] = fixed_rpm_value_;
  }

  rpm_[0] = std::max(std::min(rpm_[0], max_rpm_), min_rpm_);

  if (rpm_[0] > deadzone_propeller_neg_ && rpm_[0] < deadzone_propeller_pos_)
  {
    rpm_[0] = 0.0;
  }

  rps_ = rpm_[0] / 60;

  if (abs(rps_ * prop_pitch) < 1e-6)
  {
    tau_u_ = 0;
  }
  else
  {
    tau_u_ = 2 * fluid_density * pow(rps_, 2) * pow(D, 4) * K_T_BP * (1 - body_velocity_[0] / (rps_ * prop_pitch));
  }

  if (rps_ < 0)
  {
    tau_u_ = -tau_u_;
  }

  if (std::abs(body_velocity_[0]) < 1e-6)
  {
    RCLCPP_WARN(get_logger(), "Surge velocity is very low check if vehicle is moving");
  }
  else
  {
    sideslip_angle_ = atan2(body_velocity_[1], body_velocity_[0]);
  }

  course_angle_ = orientation_[2] + sideslip_angle_;
  speed_ = sqrt(pow(body_velocity_[0], 2) + pow(body_velocity_[1], 2));

  rudder_velocity_[0] = speed_ * std::cos(course_angle_) + orientation_rate_[2] * l_ * std::sin(orientation_[2]);
  rudder_velocity_[1] = speed_ * std::sin(course_angle_) - orientation_rate_[2] * l_ * std::cos(orientation_[2]);

  gamma_ = atan2(rudder_velocity_[1], rudder_velocity_[0]) - orientation_[2];

  gamma_ = wrapToPi(gamma_);

  rudder_angle_ = std::max(std::min(rudder_angle_, max_rudder_angle_), min_rudder_angle_);

  alpha_ = rudder_angle_ + gamma_;
  alpha_ = wrapToPi(alpha_);

  mod_ruder_velocity_ = sqrt(pow(rudder_velocity_[0], 2) + pow(rudder_velocity_[1], 2));
  lift_ = K_L * alpha_ * pow(mod_ruder_velocity_, 2);
  drag_ = (K_D0 + K_D1 * pow(alpha_, 2)) * pow(mod_ruder_velocity_, 2);

  body_acceleration_[0] = (1 / m_u) * (tau_u_ + m_v * body_velocity_[1] * orientation_rate_[2] + X_u * body_velocity_[0] + X_uu * std::abs(body_velocity_[0]) * body_velocity_[0] - drag_ * std::cos(gamma_) + lift_ * std::sin(gamma_));
  tau_r_ = l_ * (lift_ * std::cos(gamma_) + drag_ * std::sin(gamma_));
  angular_acceleration_[2] = (1 / m_r) * ((m_u - m_v) * body_velocity_[0] * body_velocity_[1] + N_r * orientation_rate_[2] + N_rr * std::abs(orientation_rate_[2]) * orientation_rate_[2] + tau_r_);

  body_velocity_[0] += body_acceleration_[0] * node_period_; 
  body_velocity_[1] += body_acceleration_[1] * node_period_;
  orientation_rate_[2] += angular_acceleration_[2] * node_period_;

  // KINEMATICS

  position_dot_[0] = body_velocity_[0] * std::cos(orientation_[2]) - body_velocity_[1] * std::sin(orientation_[2]) + curr_vel_ * std::cos(curr_dir_);
  position_dot_[1] = body_velocity_[0] * std::sin(orientation_[2]) + body_velocity_[1] * std::cos(orientation_[2]) + curr_vel_ * std::sin(curr_dir_);

  position_[0] += position_dot_[0] * node_period_;
  position_[1] += position_dot_[1] * node_period_;

  body_velocity_[0] = position_dot_[0] * std::cos(orientation_[2]) + position_dot_[1] * std::sin(orientation_[2]);
  body_velocity_[1] = -position_dot_[0] * std::sin(orientation_[2]) + position_dot_[1] * std::cos(orientation_[2]);

  orientation_[2] += orientation_rate_[2] * node_period_;
  orientation_[2] = wrapToPi(orientation_[2]);
}

void MagicElectricSim::publishMeasurements()
{
  const auto stamp = get_clock()->now();
  // Current position in UTM (absolute)
  double north = position_[0] + northing_;
  double east  = position_[1] + easting_;
  double depth = position_[2];

  const Eigen::Vector3d current_ned(
    curr_vel_ * std::cos(curr_dir_),
    curr_vel_ * std::sin(curr_dir_),
    0.0);
  const Eigen::Vector3d velocity_over_ground_ned(position_dot_[0], position_dot_[1], position_dot_[2]);
  const Eigen::Vector3d velocity_through_water_ned = velocity_over_ground_ned - current_ned;
  const double yaw = orientation_[2];
  const Eigen::Vector3d velocity_through_water_body(
    velocity_through_water_ned.x() * std::cos(yaw) + velocity_through_water_ned.y() * std::sin(yaw),
    -velocity_through_water_ned.x() * std::sin(yaw) + velocity_through_water_ned.y() * std::cos(yaw),
    velocity_through_water_ned.z());
  if (gnss_activate_) {
    const double north_meas = north + (noise_activate_ ? randn(pos_bias[0], pos_variance[0]) : 0.0);
    const double east_meas = east + (noise_activate_ ? randn(pos_bias[1], pos_variance[1]) : 0.0);

    geometry_msgs::msg::Vector3Stamped utm_msg;
    utm_msg.header.stamp = stamp;
    utm_msg.vector.x = north_meas;
    utm_msg.vector.y = east_meas;
    utm_msg.vector.z = static_cast<double>(utm_zone_);
    utm_ned_pub_->publish(utm_msg);

    double latitude = 0.0;
    double longitude = 0.0;
    GeographicLib::UTMUPS::Reverse(utm_zone_, northp_, east_meas, north_meas, latitude, longitude);

    sensor_msgs::msg::NavSatFix gnss_msg;
    gnss_msg.header.stamp = stamp;
    gnss_msg.latitude = latitude;
    gnss_msg.longitude = longitude;
    gnss_msg.altitude = -depth;
    gnss_pub_->publish(gnss_msg);

    if (gnss_velocity_over_ground_activate_) {
      geometry_msgs::msg::Vector3Stamped vel_msg;
      vel_msg.header.stamp = stamp;
      vel_msg.header.frame_id = frame_prefix_ + "gnss_link";
      vel_msg.vector.x = velocity_over_ground_ned.x() + (noise_activate_ ? randn(vel_bias[0], vel_variance[0]) : 0.0);
      vel_msg.vector.y = velocity_over_ground_ned.y() + (noise_activate_ ? randn(vel_bias[1], vel_variance[1]) : 0.0);
      vel_msg.vector.z = velocity_over_ground_ned.z() + (noise_activate_ ? randn(vel_bias[2], vel_variance[2]) : 0.0);
      velocity_over_ground_pub_->publish(vel_msg);
    }
  }

  geometry_msgs::msg::Vector3Stamped fluid_vel_msg;
  fluid_vel_msg.header.stamp = stamp;
  fluid_vel_msg.header.frame_id = frame_prefix_ + "dvl_link";
  fluid_vel_msg.vector.x = velocity_through_water_body.x() + (noise_activate_ ? randn(vel_bias[0], vel_variance[0]) : 0.0);
  fluid_vel_msg.vector.y = velocity_through_water_body.y() + (noise_activate_ ? randn(vel_bias[1], vel_variance[1]) : 0.0);
  fluid_vel_msg.vector.z = velocity_through_water_body.z() + (noise_activate_ ? randn(vel_bias[2], vel_variance[2]) : 0.0);
  velocity_through_water_pub_->publish(fluid_vel_msg);

  if (depth_sensor_activate_) {
    farol2_interfaces::msg::Depth depth_msg;
    depth_msg.header.stamp = stamp;
    depth_msg.header.frame_id = frame_prefix_ + "depth_link";
    depth_msg.depth = depth + (noise_activate_ ? randn(pos_bias[2], pos_variance[2]) : 0.0);
    depth_msg.depth_variance = noise_activate_ ? pos_variance[2] : 0.0;
    depth_pub_->publish(depth_msg);
  }

  if (imu_activate_) {
    const double roll = orientation_[0] + (noise_activate_ ? randn(ori_bias[0], ori_variance[0]) : 0.0);
    const double pitch = orientation_[1] + (noise_activate_ ? randn(ori_bias[1], ori_variance[1]) : 0.0);
    const double yaw_meas = orientation_[2] + (noise_activate_ ? randn(ori_bias[2], ori_variance[2]) : 0.0);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw_meas);
    q.normalize();

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.orientation = tf2::toMsg(q);
    imu_msg.angular_velocity.x = orientation_rate_[0] + (noise_activate_ ? randn(ori_rate_bias[0], ori_rate_variance[0]) : 0.0);
    imu_msg.angular_velocity.y = orientation_rate_[1] + (noise_activate_ ? randn(ori_rate_bias[1], ori_rate_variance[1]) : 0.0);
    imu_msg.angular_velocity.z = orientation_rate_[2] + (noise_activate_ ? randn(ori_rate_bias[2], ori_rate_variance[2]) : 0.0);
    imu_msg.linear_acceleration.x = body_acceleration_[0];
    imu_msg.linear_acceleration.y = body_acceleration_[1];
    imu_msg.linear_acceleration.z = body_acceleration_[2];
    imu_pub_->publish(imu_msg);
  }
}

void MagicElectricSim::publishWorldTransform(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = frame_prefix_ + "base_link";
  transform.transform.translation.x = position_[0];
  transform.transform.translation.y = position_[1];
  transform.transform.translation.z = position_[2];

  tf2::Quaternion q;
  q.setRPY(orientation_[0], orientation_[1], orientation_[2]);
  q.normalize();
  transform.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(transform);
}


double MagicElectricSim::randn(double mu, double sigma)
{
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
  if (call) { call = !call; return mu + sigma * X2; }
  do {
    U1 = -1.0 + ((double)rand() / RAND_MAX) * 2.0;
    U2 = -1.0 + ((double)rand() / RAND_MAX) * 2.0;
    W  = U1*U1 + U2*U2;
  } while (W >= 1.0 || W == 0.0);
  mult = sqrt(-2.0 * log(W) / W);
  X1 = U1 * mult; X2 = U2 * mult;
  call = !call;
  return mu + sigma * X1;
}

int main(int argc, char **argv)
{
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagicElectricSim>());
  rclcpp::shutdown();
  return 0;
}
