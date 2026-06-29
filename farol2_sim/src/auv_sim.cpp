#include <auv_sim.hpp>

#include <algorithm>
#include <stdexcept>

/* Constructor */
AuvSim::AuvSim() : Node("auv_sim"){
  clock_ = this->get_clock();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    get_node_base_interface(),
    get_node_logging_interface(),
    get_node_parameters_interface(),
    get_node_topics_interface());

  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();
}

/* Destructor */
AuvSim::~AuvSim() {
  /* Stop the timer */
  if (timer_) {
    timer_->cancel();
  }
  if (tf_initialisation_timer_) {
    tf_initialisation_timer_->cancel();
  }
}

/**
 * @brief Load parameters
 */
void AuvSim::loadParams() {

  freq_ = declare_parameter<int>("node_frequency");
  node_period_ = 1.0/freq_;
  frame_prefix_ = declare_parameter<std::string>("frame_prefix", "");

  fluid_density = declare_parameter<double>("environment.fluid_density");

  mass = declare_parameter<double>("vehicle.mass");
  zg = declare_parameter<double>("vehicle.zg");
  vehicle_density = declare_parameter<double>("vehicle.vehicle_density");

  inertia = declare_parameter<std::vector<double>>("vehicle.inertia_tensor");
  Dl = declare_parameter<std::vector<double>>("vehicle.linear_damping_tensor");
  Dq = declare_parameter<std::vector<double>>("vehicle.quadratic_damping_tensor");
  added_mass = declare_parameter<std::vector<double>>("vehicle.added_mass_tensor");

  allocation_flat = declare_parameter<std::vector<double>>(
    "vehicle.actuators.allocation_matrix", std::vector<double>{});
  base_frame_ = declare_parameter<std::string>("vehicle.actuators.base_frame", "");
  thruster_frames_ = declare_parameter<std::vector<std::string>>(
    "vehicle.actuators.frames", std::vector<std::string>{});
  lump_pos = declare_parameter<std::vector<double>>("vehicle.actuators.lump_param_positive");
  lump_neg = declare_parameter<std::vector<double>>("vehicle.actuators.lump_param_negative");
  minmax_input = declare_parameter<std::vector<double>>("vehicle.actuators.min_max_thruster_input");

  thruster_pole = declare_parameter<double>("vehicle.actuators.pole");
  thruster_delay = declare_parameter<double>("vehicle.actuators.delay");
  double configured_sampling_period = declare_parameter<double>("vehicle.actuators.period");
  if (std::abs(configured_sampling_period - node_period_) > 1e-9) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring vehicle.actuators.period=%.6f because the AUV thruster model is updated at node_period=%.6f.",
      configured_sampling_period,
      node_period_);
  }
  sampling_period = node_period_;

  disturbance_mean = declare_parameter<std::vector<double>>("environment.current.mean");
  disturbance_sigma = declare_parameter<std::vector<double>>("environment.current.sigma");
  disturbance_min = declare_parameter<std::vector<double>>("environment.current.minimum");
  disturbance_max = declare_parameter<std::vector<double>>("environment.current.maximum");

  // Sensor params
  gnss_activate_         = declare_parameter<bool>("sensor.gnss");
  depth_sensor_activate_ = declare_parameter<bool>("sensor.depth_sensor");
  imu_activate_          = declare_parameter<bool>("sensor.imu");
  noise_activate_        = declare_parameter<bool>("sensor.noise.activate");

  auto p_bias = declare_parameter<std::vector<double>>("sensor.noise.position.bias");
  auto p_var  = declare_parameter<std::vector<double>>("sensor.noise.position.variance");
  auto o_bias = declare_parameter<std::vector<double>>("sensor.noise.orientation.bias");
  auto o_var  = declare_parameter<std::vector<double>>("sensor.noise.orientation.variance");
  auto v_bias = declare_parameter<std::vector<double>>("sensor.noise.body_velocity.bias");
  auto v_var  = declare_parameter<std::vector<double>>("sensor.noise.body_velocity.variance");
  auto fv_bias = declare_parameter<std::vector<double>>("sensor.noise.fluid_velocity.bias");
  auto fv_var  = declare_parameter<std::vector<double>>("sensor.noise.fluid_velocity.variance");
  auto r_bias = declare_parameter<std::vector<double>>("sensor.noise.orientation_rate.bias");
  auto r_var  = declare_parameter<std::vector<double>>("sensor.noise.orientation_rate.variance");
  for (int i = 0; i < 3; ++i) {
    pos_bias[i] = p_bias[i]; pos_variance[i] = p_var[i];
    ori_bias[i] = o_bias[i]; ori_variance[i] = o_var[i];
    vel_bias[i] = v_bias[i]; vel_variance[i] = v_var[i];
    fluid_vel_bias[i] = fv_bias[i]; fluid_vel_variance[i] = fv_var[i];
    ori_rate_bias[i] = r_bias[i]; ori_rate_variance[i] = r_var[i];
  }

  initial_position_ = declare_parameter<std::vector<double>>("initial_state.position");
  initial_body_velocity_ = declare_parameter<std::vector<double>>("initial_state.body_velocity");
  initial_orientation_ = declare_parameter<std::vector<double>>("initial_state.orientation");
  initial_orientation_rate_ = declare_parameter<std::vector<double>>("initial_state.orientation_rate");
  double originLat_ = initial_position_[0];
  double originLon_ = initial_position_[1];
  GeographicLib::UTMUPS::Forward(originLat_, originLon_, utm_zone_, northp_, easting_, northing_);

  const auto apply_frame_prefix = [this](const std::string & frame) {
    if (frame_prefix_.empty() || frame.rfind(frame_prefix_, 0) == 0) {
      return frame;
    }
    return frame_prefix_ + frame;
  };

  if (!base_frame_.empty() && !thruster_frames_.empty()) {
    base_frame_ = apply_frame_prefix(base_frame_);
    for (auto & frame : thruster_frames_) {
      if (frame.empty()) {
        throw std::runtime_error("vehicle.actuators.frames cannot contain empty frame names");
      }
      frame = apply_frame_prefix(frame);
    }

    use_tf_allocation_ = true;
    return;
  }

  if (allocation_flat.empty() || allocation_flat.size() % 6 != 0) {
    throw std::runtime_error(
      "auv_sim needs either vehicle.actuators.{base_frame, frames} "
      "or a vehicle.actuators.allocation_matrix with a multiple of 6 values");
  }

  size_t n_thrusters = allocation_flat.size() / 6;
  Eigen::MatrixXd allocation_matrix(n_thrusters, 6);
  for (size_t i = 0; i < n_thrusters; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      allocation_matrix(i, j) = allocation_flat[i * 6 + j];
    }
  }

  initialiseAuv(allocation_matrix);
}

void AuvSim::initialiseAuv(const Eigen::MatrixXd &allocation_matrix) {
  Eigen::Vector3d inertia_tensor(inertia[0], inertia[1], inertia[2]);

  Eigen::Matrix<double, 6, 1> Dl_tensor, Dq_tensor, added_mass_tensor;
  for (int i = 0; i < 6; ++i) {
    Dl_tensor(i) = Dl[i];
    Dq_tensor(i) = Dq[i];
    added_mass_tensor(i) = added_mass[i];
  }

  rpm_ = Eigen::VectorXd::Zero(allocation_matrix.rows());

  Eigen::Vector3d lump_pos_vec(lump_pos[0], lump_pos[1], lump_pos[2]);
  Eigen::Vector3d lump_neg_vec(lump_neg[0], lump_neg[1], lump_neg[2]);
  Eigen::Vector2d minmax_vec(minmax_input[0], minmax_input[1]);
  Eigen::Vector3d dist_mean(disturbance_mean[0], disturbance_mean[1], disturbance_mean[2]);
  Eigen::Vector3d dist_sigma(disturbance_sigma[0], disturbance_sigma[1], disturbance_sigma[2]);
  Eigen::Vector3d dist_min(disturbance_min[0], disturbance_min[1], disturbance_min[2]);
  Eigen::Vector3d dist_max(disturbance_max[0], disturbance_max[1], disturbance_max[2]);


  
  auv_ = std::make_unique<AUV>(
    mass,
    fluid_density,
    zg,
    vehicle_density,
    inertia_tensor,
    Dl_tensor,
    Dq_tensor,
    added_mass_tensor,
    allocation_matrix,
    lump_pos_vec,
    lump_neg_vec,
    minmax_vec,
    thruster_pole,
    thruster_delay,
    sampling_period,
    dist_mean,
    dist_sigma,
    dist_min,
    dist_max
  );

  State initial_state;
  initial_state.eta1 << 0.0, 0.0, initial_position_[2];
  initial_state.eta2 << initial_orientation_[0], initial_orientation_[1], initial_orientation_[2];
  initial_state.v1 << initial_body_velocity_[0], initial_body_velocity_[1], initial_body_velocity_[2];
  initial_state.v2 << initial_orientation_rate_[0], initial_orientation_rate_[1], initial_orientation_rate_[2];
  auv_->setState(initial_state);
}

void AuvSim::initialiseAuvFromTF() {
  if (auv_ || !use_tf_allocation_) {
    return;
  }

  std::vector<ThrusterGeometry> geometry;
  if (!buildThrusterGeometryFromTF(
      *tf_buffer_,
      *clock_,
      get_logger(),
      base_frame_,
      thruster_frames_,
      geometry)) {
    return;
  }

  initialiseAuv(buildSimulatorThrusterMatrix(geometry));
  RCLCPP_INFO(
    get_logger(),
    "AUV sim allocation ready with %zu TF thruster frames using base frame '%s'.",
    geometry.size(),
    base_frame_.c_str());

  if (tf_initialisation_timer_) {
    tf_initialisation_timer_->cancel();
  }
}

/**
 * @brief Initialise Subscribers
 */
void AuvSim::initialiseSubscribers() {


  rpm_sub_  = create_subscription<farol2_interfaces::msg::ThrusterRPM>(
                          TOPIC_SUB_RPM_COMMAND, 
                          1, std::bind(&AuvSim::rpmCallback, this, std::placeholders::_1));
  return;
}


/**
 * @brief Initialise Publishers
 */
void AuvSim::initialisePublishers() {
  position_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_POSITION, 1);
  body_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_BODY_VELOCITY, 1);
  orientation_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_ORIENTATION, 1);
  orientation_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_ORIENTATION_RATE, 1);
  body_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_BODY_ACCELERATION, 1);
  angular_acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      TOPIC_PUB_ANGULAR_ACCELERATION, 1);

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
  depth_pub_ = create_publisher<std_msgs::msg::Float32>(
      TOPIC_PUB_DEPTH, 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return;
}

/**
 * @brief Initialise Services
 */
void AuvSim::initialiseServices() {
  /* Service servers */
  /* ... */

  /* service clients */
  /* ... */

  return;
}

/**
 * @brief Initialise Timers
 */
void AuvSim::initialiseTimers() {
  if (use_tf_allocation_ && !auv_) {
    tf_initialisation_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&AuvSim::initialiseAuvFromTF, this));
  }

  timer_ = create_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(std::llround(node_period_ * 1e9))),
      std::bind(&AuvSim::timerCallback, this));
  
}


/**
 * @brief Timer callback for this node.
 *        Where the algorithms will constantly run.
 */

void AuvSim::rpmCallback(const farol2_interfaces::msg::ThrusterRPM::SharedPtr msg){

  const int count = std::min<int>(rpm_.size(), msg->rpm.size());
  for(int i=0; i < count; i++) {
    rpm_[i] = msg->rpm[i];
  }
}

void AuvSim::timerCallback() {
  if (!auv_) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Timer callback triggered");
  const auto stamp = get_clock()->now();
  auv_->update(node_period_, rpm_);

  geometry_msgs::msg::Vector3 pos_msg, body_vel_msg, ori_msg, ori_rate_msg, body_acc_msg, ang_acc_msg;

  pos_msg.x = auv_->getX();
  pos_msg.y = auv_->getY();
  pos_msg.z = auv_->getZ();
  position_pub_->publish(pos_msg);

  body_vel_msg.x = auv_->getSurge();
  body_vel_msg.y = auv_->getSway();
  body_vel_msg.z = auv_->getHeave();
  body_velocity_pub_->publish(body_vel_msg);

  ori_msg.x = rad_to_deg(auv_->getRoll());
  ori_msg.y = rad_to_deg(auv_->getPitch());
  ori_msg.z = rad_to_deg(auv_->getYaw());
  orientation_pub_->publish(ori_msg);

  ori_rate_msg.x = rad_to_deg(auv_->getRollRate());
  ori_rate_msg.y = rad_to_deg(auv_->getPitchRate());
  ori_rate_msg.z = rad_to_deg(auv_->getYawRate());
  orientation_rate_pub_->publish(ori_rate_msg);

  body_acc_msg.x = auv_->getSurgeDot();
  body_acc_msg.y = auv_->getSwayDot();
  body_acc_msg.z = auv_->getHeaveDot();
  body_acceleration_pub_->publish(body_acc_msg);

  ang_acc_msg.x = auv_->getRollRateDot();
  ang_acc_msg.y = auv_->getPitchRateDot();
  ang_acc_msg.z = auv_->getYawRateDot();
  angular_acceleration_pub_->publish(ang_acc_msg);

  publishWorldTransform(stamp);
  publishMeasurements();

  return;
}

void AuvSim::publishMeasurements()
{
  const auto stamp = get_clock()->now();
  double north = auv_->getX() + northing_;
  double east  = auv_->getY() + easting_;
  double depth = auv_->getZ();
  Eigen::Vector3d fluid_velocity_body(auv_->getSurge(), auv_->getSway(), auv_->getHeave());
  Eigen::Vector3d ocean_current_inertial = auv_->getOceanCurrent();
  Eigen::Matrix3d body_to_inertial = rotationBodyToInertial(auv_->getRoll(), auv_->getPitch(), auv_->getYaw());
  Eigen::Vector3d inertial_velocity = body_to_inertial * fluid_velocity_body + ocean_current_inertial;
  Eigen::Vector3d fluid_velocity = body_to_inertial.transpose() * (inertial_velocity - ocean_current_inertial);



  
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
  }

  geometry_msgs::msg::Vector3Stamped vel_msg;
  vel_msg.header.stamp = stamp;
  vel_msg.vector.x = inertial_velocity.x() + (noise_activate_ ? randn(vel_bias[0], vel_variance[0]) : 0.0);
  vel_msg.vector.y = inertial_velocity.y() + (noise_activate_ ? randn(vel_bias[1], vel_variance[1]) : 0.0);
  vel_msg.vector.z = inertial_velocity.z() + (noise_activate_ ? randn(vel_bias[2], vel_variance[2]) : 0.0);
  velocity_over_ground_pub_->publish(vel_msg);

  geometry_msgs::msg::Vector3Stamped fluid_vel_msg;
  fluid_vel_msg.header.stamp = stamp;
  fluid_vel_msg.vector.x = fluid_velocity.x() + (noise_activate_ ? randn(fluid_vel_bias[0], fluid_vel_variance[0]) : 0.0);
  fluid_vel_msg.vector.y = fluid_velocity.y() + (noise_activate_ ? randn(fluid_vel_bias[1], fluid_vel_variance[1]) : 0.0);
  fluid_vel_msg.vector.z = fluid_velocity.z() + (noise_activate_ ? randn(fluid_vel_bias[2], fluid_vel_variance[2]) : 0.0);
  velocity_through_water_pub_->publish(fluid_vel_msg);

  if (depth_sensor_activate_) {
    std_msgs::msg::Float32 depth_msg;
    depth_msg.data = static_cast<float>(depth + (noise_activate_ ? randn(pos_bias[2], pos_variance[2]) : 0.0));
    depth_pub_->publish(depth_msg);
  }

  if (imu_activate_) {
    const double roll = auv_->getRoll() + (noise_activate_ ? randn(ori_bias[0], ori_variance[0]) : 0.0);
    const double pitch = auv_->getPitch() + (noise_activate_ ? randn(ori_bias[1], ori_variance[1]) : 0.0);
    const double yaw = auv_->getYaw() + (noise_activate_ ? randn(ori_bias[2], ori_variance[2]) : 0.0);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.orientation = tf2::toMsg(q);
    imu_msg.angular_velocity.x = auv_->getRollRate() + (noise_activate_ ? randn(ori_rate_bias[0], ori_rate_variance[0]) : 0.0);
    imu_msg.angular_velocity.y = auv_->getPitchRate() + (noise_activate_ ? randn(ori_rate_bias[1], ori_rate_variance[1]) : 0.0);
    imu_msg.angular_velocity.z = auv_->getYawRate() + (noise_activate_ ? randn(ori_rate_bias[2], ori_rate_variance[2]) : 0.0);
    imu_msg.linear_acceleration.x = auv_->getSurgeDot();
    imu_msg.linear_acceleration.y = auv_->getSwayDot();
    imu_msg.linear_acceleration.z = auv_->getHeaveDot();
    imu_pub_->publish(imu_msg);
  }
}

void AuvSim::publishWorldTransform(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = frame_prefix_ + "base_link";
  transform.transform.translation.x = auv_->getX();
  transform.transform.translation.y = auv_->getY();
  transform.transform.translation.z = auv_->getZ();

  tf2::Quaternion q;
  q.setRPY(auv_->getRoll(), auv_->getPitch(), auv_->getYaw());
  q.normalize();
  transform.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(transform);
}

double AuvSim::randn(double mu, double sigma)
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

/**
 * @brief Main function
 */



AUV::AUV(double mass,
          double fluid_density,
          double zg,
          double vehicle_density,
          const Eigen::Vector3d &inertia_tensor,
          const Eigen::Matrix<double, 6, 1> &linear_damping_tensor,
          const Eigen::Matrix<double, 6, 1> &quadratic_damping_tensor,
          const Eigen::Matrix<double, 6, 1> &added_mass_tensor,
          const Eigen::MatrixXd &allocation_matrix,
          const Eigen::Vector3d &lump_param_positive,
          const Eigen::Vector3d &lump_param_negative,
          const Eigen::Vector2d &min_max_thruster_input,
          double thrusters_pole,
          double thrusters_delay,
          double sampling_period,
          const Eigen::Vector3d &disturbance_mean,
          const Eigen::Vector3d &disturbance_sigma,
          const Eigen::Vector3d &disturbance_minimum,
          const Eigen::Vector3d &disturbance_maximum): mass_(mass), fluid_density_(fluid_density), zg_(zg),
                lump_param_positive_(lump_param_positive),
                lump_param_negative_(lump_param_negative),
                min_max_thruster_input_(min_max_thruster_input),
                thrusters_pole_(thrusters_pole),
                thrusters_delay_(thrusters_delay),
                sampling_period_(sampling_period),
                disturbance_minimum_(disturbance_minimum),
                disturbance_maximum_(disturbance_maximum) {

    /* Compute the radius of a sphere with equivalent density to the vehicle */
    this->radius_ = std::pow(mass/((4.0/3.0)*M_PI*vehicle_density), 1.0/3.0);

    /* Create a 6x6 diagonal rigid body mass matrix */
    Eigen::Matrix<double, 6, 6> mass_matrix = Eigen::Matrix<double, 6, 6>::Zero(6, 6);
    for(int i=0; i < 3; i++) mass_matrix(i, i) = mass;
    for(int i=3; i < 6; i++) mass_matrix(i, i) = inertia_tensor(i-3);

    /* Create a 6x6 diagonal added mass matrix */
    Eigen::Matrix<double, 6, 6> added_mass_matrix;
    added_mass_matrix = added_mass_tensor.asDiagonal();

    /* Compute the 6x6 diagonal total mass matrix */
    this->M_ = mass_matrix + added_mass_matrix;
    this->M_inv_ = this->M_.inverse();
    this->M11_ = this->M_.block(0, 0, 3, 3);
    this->M12_ = this->M_.block(0, 3, 3, 3);
    this->M21_ = this->M_.block(3, 0, 3, 3);
    this->M22_ = this->M_.block(3, 3, 3, 3);

    /* Create a 6x6 diagonal linear damping matrix */
    this->Dl_ = linear_damping_tensor.asDiagonal();

    /* Create a 6x6 diagonal quadratic damping matrix */
    this->Dq_ = quadratic_damping_tensor.asDiagonal();

    /* Save the allocation matrix */
    this->allocation_matrix_ = allocation_matrix;

    /* Initialize the thrusters */
    this->number_of_delays_inputs_ = int(std::round(this->thrusters_delay_ / this->sampling_period_));

    /* Initialize the thrusters input vectors with zeros (used as circular buffer) */
    double number_thrusters = this->allocation_matrix_.rows();
    for(int i=0; i < this->number_of_delays_inputs_; i++) {
        /* Save a vector full of zeros with the size corresponding to the number of thrusters */
        this->thrusters_inputs_.emplace_back(Eigen::VectorXd::Zero(number_thrusters));
    }

    /* Initialize the thrusters last output vector with zeros */
    this->last_thruster_output_ = Eigen::VectorXd::Zero(number_thrusters);

    /* Initiate the ocean disturbances random process distributions */
    this->generator_ = std::default_random_engine();
    for(int i=0; i < 3; i++) distributions_[i] = std::normal_distribution<double>(disturbance_mean(i), disturbance_sigma(i));
}

void AUV::update(double dt, const Eigen::VectorXd &thrust) {

    /* Verify if the size of the thrust vector corresponds to the number of thrusters of the vehicle */
    if(thrust.size() != this->getNumberThrusters()) throw std::invalid_argument("Thrust vector size should be " + std::to_string(this->getNumberThrusters()));

    /* Verify that dt (time difference is positive) */
    if(dt < 0) throw std::invalid_argument("dt parameter should be positive");

    /* Propagate the desired RPM through the thruster model */
    Eigen::VectorXd applied_thrust;
    applied_thrust = this->applyThrustersModel(thrust);

    /* Convert the applied thrust in the motors to forces and torques applied to the rigid body */
    Eigen::Matrix<double, 6, 1> applied_forces_and_torques;
    applied_forces_and_torques = this->convertThrustToGeneralForces(applied_thrust);

    /* Compute the ocean disturbances */
    Eigen::Vector3d ocean_disturbances;
    ocean_disturbances = this->computeOceanDisturbances();
    this->last_ocean_disturbances_ = ocean_disturbances;

    /* Compute the dynamics of the rigid body - linear and angular acceleration in body frame */
    Eigen::Vector3d v1_dot, v2_dot;
    std::tie(v1_dot, v2_dot) = this->updateDynamics(applied_forces_and_torques);
    // Store the latest accelerations
    this->last_v1_dot_ = v1_dot;
    this->last_v2_dot_ = v2_dot;

    /* Integrate the dynamics */
    Eigen::Vector3d v1(this->state_.v1), v2(this->state_.v2);
    v1 = eulerIntegration(dt, v1, v1_dot);
    v2 = eulerIntegration(dt, v2, v2_dot);

    /* Compute the kinematics of the rigid body - linear angular velocity in inertial frame */
    Eigen::Vector3d eta1_dot, eta2_dot;
    std::tie(eta1_dot, eta2_dot) = this->updateKinematics(v1, v2, ocean_disturbances);

    /* Integrate the kinematics */
    Eigen::Vector3d eta1(this->state_.eta1), eta2(this->state_.eta2);
    eta1 = eulerIntegration(dt, eta1, eta1_dot);
    eta2 = eulerIntegration(dt, eta2, eta2_dot);

    /* Wrap angles */
    for(int i=0; i<3; i++) eta2(i) = wrapTo2Pi(eta2(i));

    /* Update the state of the vehicle */
    this->state_.eta1 = eta1;
    this->state_.eta2 = eta2;
    this->state_.v1 = v1;
    this->state_.v2 = v2;
}

Eigen::MatrixXd AUV::applyThrustersModel(const Eigen::VectorXd &thrust) {

    /* Create a vector of input RPM */
    Eigen::VectorXd input_thrust = Eigen::VectorXd(thrust.size());

    /* For each thruster RPM input */
    for(int i=0; i < thrust.size(); i++) {

        /* Saturate the RPM values that go beyond the configured limits */
        input_thrust(i) = saturation(thrust(i), -this->min_max_thruster_input_(1), this->min_max_thruster_input_(1));

        /* Apply an RPM dead-zone */
        if (input_thrust(i) < this->min_max_thruster_input_(0) && input_thrust(i) > -this->min_max_thruster_input_(0)) input_thrust(i) = 0.0;
    }

    /* Rotate the circular buffer of inputs (rotate to the left), and save the new input in the last element  */
    std::rotate(this->thrusters_inputs_.begin(), this->thrusters_inputs_.begin() + 1, this->thrusters_inputs_.end());
    this->thrusters_inputs_.back() = input_thrust;

    /* --- Apply the thrusters (simple-pole + delay) discrete model --- */
    /* load from the thruster buffer the desired inputs, such that: input = u[k-delay] */
    Eigen::VectorXd u_old = this->thrusters_inputs_[0];

    /* Calculate the applied y[k] using the difference equations (discretized model of simple-pole motor + delay) */
    /* y(k+1) = e^{-pole * period} * y(k) + (1-e^{-pole * period}) * u(k-number_of_delays) */
    Eigen::VectorXd y_k1, y_k;
    y_k = this->last_thruster_output_;
    y_k1 = (std::exp(-this->thrusters_pole_ * this->sampling_period_) * y_k) + ((-std::exp(-this->thrusters_pole_ * this->sampling_period_) + 1) * u_old);

    /* Saturate the output between the minimum and maximum values */
    for(int i=0; i < thrust.size(); i++) y_k1(i) = saturation(y_k1(i), -this->min_max_thruster_input_(1), this->min_max_thruster_input_(1));

    /* Save the new output to used in the next iteration */
    this->last_thruster_output_ = y_k1;

    /* Convert the output values to forces [N], using the parable curves */
    Eigen::VectorXd output_force = y_k1;
    for(int i=0; i < y_k1.size(); i++) output_force[i] = this->convertThrustToForce(y_k1[i]);

    /* Return the output force applied by each thruster */
    return output_force;
}

double AUV::convertThrustToForce(const double thrust) {

    double a, b, c;
    /* Choose the constants for the parable depending if we are on the positive or negative side of the thrust curve */
    if(thrust >= 0) {
        a = this->lump_param_positive_[0];
        b = this->lump_param_positive_[1];
        c = this->lump_param_positive_[2];
    } else {
        a = this->lump_param_negative_[0];
        b = this->lump_param_negative_[1];
        c = this->lump_param_negative_[2];
    }

    /* Apply the parable formula F = a|x|^2 + b|x| + c */
    return (a * std::pow(thrust, 2)) + (b * std::abs(thrust)) + c;
}

Eigen::Matrix<double, 6, 1> AUV::convertThrustToGeneralForces(const Eigen::VectorXd &thrust) {

    /* Inititate the matrix with all forces and torques with contributions from all vehicles*/
    Eigen::Matrix<double, 6, 1> forces_and_torques = Eigen::Matrix<double, 6, 1>::Zero();

    /* Initiate the vector of forces */
    Eigen::Vector3d total_force;
    Eigen::Vector3d total_torque;

    /* Compute the contribution of the force applied by each thruster to the total force */
    for(unsigned int i=0; i < this->getNumberThrusters(); i++) {

        /* Get the contribution of the thruster to the force in Fx, Fy and Fz */
        this->allocation_matrix_.block<1,3>(i,0);

        /* Compute the total force applied in each direction by thruster "i" */
        total_force = thrust[i] * this->allocation_matrix_.block<1,3>(i,0);

        /* Compute the total torque applied about each axis by thruster "i" */
        total_torque = this->allocation_matrix_.block<1,3>(i,3).cross(total_force);

        /* Accumulate the forces and torques from all the thrusters */
        forces_and_torques.block<3,1>(0, 0) += total_force;
        forces_and_torques.block<3,1>(3, 0) += total_torque;
    }

    return forces_and_torques;
}





Eigen::Vector3d AUV::computeOceanDisturbances() {

    Eigen::Vector3d ocean_disturbances = Eigen::Vector3d(0.0, 0.0, 0.0);

    /* Generate random numbers according to the desired mean and standard deviation for x, y and z axis */
    for(int i=0; i<3; i++) {

        // Generate the random number according to the gaussian distribution
        ocean_disturbances(i) = distributions_[i](generator_);

        // Bound the disturbances with limits
        if(ocean_disturbances(i) < disturbance_minimum_(i)) ocean_disturbances(i) = disturbance_minimum_(i);
        if(ocean_disturbances(i) > disturbance_maximum_(i)) ocean_disturbances(i) = disturbance_maximum_(i);
    }

    return ocean_disturbances;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> AUV::updateDynamics(const Eigen::Matrix<double, 6, 1> &forces_and_torques) {

    Eigen::Vector3d v1_dot, v2_dot;

    /* Compute the Coriolis Matrix (6x6) */
    Eigen::Matrix<double, 6, 6> coriolis;
    coriolis = this->computeCoriolisMatrix();

    /* Compute the Damping Matrix (6x6) */
    Eigen::Matrix<double, 6, 6> damping;
    damping = this->computeDampingTerms();

    /* Compute Gravitational Forces Vector (6x1) */
    Eigen::Matrix<double, 6, 1> hydrostatic_forces;
    hydrostatic_forces = this->computeHydrostaticForces();

    /* Compute the updated v_dot */
    Eigen::Matrix<double, 6, 1> v, v_dot;
    v << this->state_.v1, this->state_.v2;
    // Note: Damping terms already contain the (-) sign, therefore we should sum here
    v_dot = this->M_inv_ * (forces_and_torques - hydrostatic_forces - (coriolis * v) + (damping * v));

    /* Get the individual linear and angular velocity terms and return them */
    v1_dot << v_dot(0), v_dot(1), v_dot(2);
    v2_dot << v_dot(3), v_dot(4), v_dot(5);

    return std::make_tuple(v1_dot, v2_dot);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> AUV::updateKinematics(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &ocean_disturbances) {

    Eigen::Vector3d eta1_dot, eta2_dot;

    /* Compute the linear velocity in the inertial frame */
    eta1_dot = (rotationBodyToInertial(this->state_.eta2(0), this->state_.eta2(1), this->state_.eta2(2)) * v1) + ocean_disturbances;

    /* Compute the angular velocity in the inertial frame */
    eta2_dot = rotationAngularBodyToInertial(this->state_.eta2(0), this->state_.eta2(1)) * v2;

    return std::make_tuple(eta1_dot, eta2_dot);
}

Eigen::Matrix<double, 6, 6> AUV::computeCoriolisMatrix() {

    Eigen::Matrix<double, 6, 6> coriolis = Eigen::MatrixXd::Zero(6, 6);

    /* Compute the coriolis terms from the mass matrix and the velocity vector in the body frame */
    Eigen::Matrix3d s1, s2;
    s1 = computeSkewSymmetric((this->M11_ * this->state_.v1) + (this->M12_ * this->state_.v2));
    s2 = computeSkewSymmetric((this->M21_ * this->state_.v1) + (this->M22_ * this->state_.v2));

    coriolis.block(0, 3, 3, 3) = -s1;
    coriolis.block(3, 0, 3, 3) = -s1;
    coriolis.block(3, 3, 3, 3) = -s2;

    return coriolis;
}

Eigen::Matrix<double, 6, 6> AUV::computeDampingTerms() {

    Eigen::Matrix<double, 6, 6> damping, quadratic_damping;

    /* Get the state vector corresponding to linear and angular velocities in the Body frame */
    Eigen::Matrix<double, 6, 1> v, quadratic_damping_vector;
    v << this->state_.v1, this->state_.v2;

    /* Compute the damping matrix according to the formulas */
    quadratic_damping_vector = this->Dq_ * v.cwiseAbs();
    quadratic_damping = quadratic_damping_vector.asDiagonal();
    damping = this->Dl_ + quadratic_damping;

    return damping;
}

Eigen::Matrix<double, 6, 1> AUV::computeHydrostaticForces() {

    Eigen::Matrix<double, 6, 1> hydrostatic_forces;

    /* Compute the Weight force applied to the vehicle in the inertial frame */
    double W = this->mass_ * this->gravity_;

    /* Compute the volume of fluid displaced */
    double volume_fluid_displaced = this->computeVolumeFluidDisplaced();

    /* Compute the buoyancy force applied to the vehicle in the inertial frame */
    double B = this->fluid_density_ * this->gravity_ * volume_fluid_displaced;

    /* Compute the hydrostatic forces vector */
    /* NOTE: an assumption is made that xg and yg (center x and y center of gravity of the robot are 0.0) which is not
     * necessarily true. This model can be improved in the future. See Fossen's handbook to marine crafts */
    hydrostatic_forces << (W - B) * sin(this->state_.eta2(1)),
                     -(W - B) * cos(this->state_.eta2(1)) * sin(this->state_.eta2(0)),
                     -(W - B) * cos(this->state_.eta2(1)) * cos(this->state_.eta2(0)),
                      this->zg_ * W * cos(this->state_.eta2(1)) * sin(this->state_.eta2(0)),
                      this->zg_ * W * sin(this->state_.eta2(1)),
                      0.0;

    return hydrostatic_forces;
}

double AUV::computeVolumeFluidDisplaced() {

    double r = this->radius_;

    /* Assume that the vehicle is a sphere that decreases its radius when the center of mass of the vehicle is above the surface of water */
    /* NOTE: this is not the best model and should be improved in future iterations */
    if(this->state_.eta1(2) < 0.0) {
        r = this->radius_ + this->state_.eta1(2);
        if(r < 0.0) r = 0;
    }

    /* Return the volume of the sphere (which corresponds to the amount of fluid displaced by itself */
    return (4.0 / 3.0) * M_PI * std::pow(r, 3);
}


double AUV::getX() const { return state_.eta1(0); }
double AUV::getY() const { return state_.eta1(1); }
double AUV::getZ() const { return state_.eta1(2); }
double AUV::getRoll() const { return state_.eta2(0); }
double AUV::getPitch() const { return state_.eta2(1); }
double AUV::getYaw() const { return state_.eta2(2); }  
double AUV::getSurge() const { return state_.v1(0); }
double AUV::getSway() const { return state_.v1(1); }
double AUV::getHeave() const { return state_.v1(2); } 
double AUV::getRollRate() const { return state_.v2(0); }
double AUV::getPitchRate() const { return state_.v2(1); }
double AUV::getYawRate() const { return state_.v2(2); }      
double AUV::getSurgeDot() const { return last_v1_dot_.x(); }
double AUV::getSwayDot() const { return last_v1_dot_.y(); }
double AUV::getHeaveDot() const { return last_v1_dot_.z(); }
double AUV::getRollRateDot() const { return last_v2_dot_.x(); }
double AUV::getPitchRateDot() const { return last_v2_dot_.y(); }
double AUV::getYawRateDot() const { return last_v2_dot_.z(); } 
Eigen::Vector3d AUV::getOceanCurrent() const { return last_ocean_disturbances_; }



int main(int argc, char ** argv) {
  /* initialise ROS2 and start the node */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AuvSim>());
  rclcpp::shutdown();
  return 0;
}
