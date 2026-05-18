#include <sample_and_hold.hpp>

#include <algorithm>
#include <cmath>

double smooth_deadzone(double u, double dz, double sharpness)
{
    double a = std::abs(u);

    if (a < 1e-9)
        return 0.0;

    double scale = 1.0 - std::exp(-std::pow(a / dz, sharpness));

    return u * scale;
}



/* Constructor */
SampleAndHold::SampleAndHold() : Node("sample_and_hold") {
  clock_ = this->get_clock();
  loadParams();
  initialiseSubscribers();
  initialisePublishers();
  initialiseServices();
  initialiseTimers();

  x_ = Eigen::Vector2d::Zero();
  P_ = 1e-3 * Eigen::Matrix2d::Identity();
  C_ << 1, 0;
  Q_ = Eigen::Matrix2d::Identity();
  R_ << 10000.0;
  rudder_angle_ = 0.0;
  H_pos_ << 1, 0, 0, 0,
            0, 1, 0, 0;
  Q_pos_.setZero();
  Q_pos_.diagonal() << ekf_q_pos_, ekf_q_pos_, ekf_q_current_, ekf_q_current_;
  R_pos_.setIdentity();
  R_pos_ *= ekf_r_pos_;
}

/* Destructor */
SampleAndHold::~SampleAndHold() {}

/**
 * @brief Load parameters
 */
void SampleAndHold::loadParams() {
  neglect_current_ = declare_parameter<bool>("neglect_current");
  node_frequency_ = declare_parameter<double>("node_frequency");  
  course_angle_cutoff_frequency_ = 2.0 * M_PI * declare_parameter<double>("course_cf.cutoff_frequency");
  use_yaw_rate_lpf_ = declare_parameter<bool>("yaw_rate_lpf.use", false);
  yaw_rate_lpf_.configure(declare_parameter<double>("yaw_rate_lpf.omega_cutoff"),
                          1/node_frequency_, 
                          declare_parameter<int>("yaw_rate_lpf.order"), 
                          declare_parameter<std::string>("yaw_rate_lpf.design"), 
                          declare_parameter<std::string>("yaw_rate_lpf.method"));
  use_yaw_rate_kf_ = declare_parameter<bool>("yaw_rate_kf.use", false);
  use_course_cf_ = declare_parameter<bool>("course_cf.use", false);

  use_yaw_rate_notch_filter_ = declare_parameter<bool>("yaw_rate_notch_filter.use", false);
  yaw_rate_notch_f0_hz_ = declare_parameter<double>("yaw_rate_notch_filter.f0_hz", 0.32);
  yaw_rate_notch_q_ = declare_parameter<double>("yaw_rate_notch_filter.q", 1.0);

  if (use_yaw_rate_notch_filter_) {
    try {
      yaw_rate_notch_filter_.configure(yaw_rate_notch_f0_hz_, yaw_rate_notch_q_);
      yaw_rate_notch_filter_.reset();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Yaw-rate notch filter configuration failed: %s", e.what());
      use_yaw_rate_notch_filter_ = false;
    }
  }
  
  // Kalman filter parameters
  time_constant_ = declare_parameter<double>("kalman.time_constant", 1.0);
  rudder_gain_ = declare_parameter<double>("kalman.rudder_gain", 1.0);
  process_noise_q_ = declare_parameter<double>("kalman.process_noise", 0.01);
  measurement_noise_r_ = declare_parameter<double>("kalman.measurement_noise", 0.1);
  Q_ = process_noise_q_ * Eigen::Matrix2d::Identity();
  R_ << measurement_noise_r_;

  use_position_ekf_ = declare_parameter<bool>("position_ekf.use", false);
  gps_timeout_s_ = declare_parameter<double>("position_ekf.gps_timeout_s", 3.0);
  rpm_timeout_s_ = declare_parameter<double>("position_ekf.rpm_timeout_s", 1.0);
  ekf_q_pos_ = declare_parameter<double>("position_ekf.process_noise_pos", 0.05);
  ekf_q_current_ = declare_parameter<double>("position_ekf.process_noise_current", 0.01);
  ekf_r_pos_ = declare_parameter<double>("position_ekf.measurement_noise_pos", 4.0);
  ekf_p0_pos_ = declare_parameter<double>("position_ekf.init_cov_pos", 25.0);
  ekf_p0_current_ = declare_parameter<double>("position_ekf.init_cov_current", 1.0);
  rpm_min_ = declare_parameter<double>("position_ekf.rpm_min", -2000.0);
  rpm_max_ = declare_parameter<double>("position_ekf.rpm_max", 2000.0);
  rpm_rate_limit_ = declare_parameter<double>("position_ekf.rpm_rate_limit", 200.0);
  rho_ = declare_parameter<double>("position_ekf.rho", 1025.0);
  prop_pitch_ = declare_parameter<double>("position_ekf.prop_pitch", 0.381);
  prop_diameter_ = declare_parameter<double>("position_ekf.prop_diameter", 0.4318);
  k_t_bp_ = declare_parameter<double>("position_ekf.k_t_bp", 0.061461915);
  m_u_ = declare_parameter<double>("position_ekf.m_u", 3100.0);
  x_u_ = declare_parameter<double>("position_ekf.x_u", 0.0);
  x_uu_ = declare_parameter<double>("position_ekf.x_uu", -96.2270);
  override_velocity_ = declare_parameter<double>("position_ekf.override_velocity", 0.0);
  override_rpms_ = declare_parameter<double>("position_ekf.override_rpms", 600.0);
  override_timeout_s_ = declare_parameter<double>("position_ekf.override_timeout_s", 5.0);
}

/**
 * @brief Initialise Subscribers
 */
void SampleAndHold::initialiseSubscribers() {
  measurement_sub_ = create_subscription<farol2_interfaces::msg::Measurement>(
    declare_parameter<std::string>("topics.subscribers.measurement"),
    rclcpp::QoS(10),
    [this](farol2_interfaces::msg::Measurement::SharedPtr msg){measurement_callback(msg);});

  rudder_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/magicelectric0/drivers/can_instrumentation/rudder_angle",
    rclcpp::QoS(1),
    [this](std_msgs::msg::Float32::SharedPtr msg) { rudder_angle_ = farol2_utils::deg2rad(msg->data); });

  rpm_command_sub_ = create_subscription<farol2_allocation::msg::ThrusterRPM>(
    declare_parameter<std::string>("topics.subscribers.rpm_command", "/magicelectric0/actuation/rpm_command"),
    rclcpp::QoS(1),
    [this](farol2_allocation::msg::ThrusterRPM::ConstSharedPtr msg) { rpm_command_callback(msg); });
 
  return;
}

void SampleAndHold::rpm_command_callback(farol2_allocation::msg::ThrusterRPM::ConstSharedPtr msg) {
  if (msg->rpm.empty()) {
    return;
  }
  latest_rpm_command_ = msg->rpm[0];
  last_rpm_command_time_s_ = clock_->now().seconds();
}

/**
 * @brief Initialise Publishers
 */
void SampleAndHold::initialisePublishers() {
  state_pub_ = create_publisher<farol2_interfaces::msg::NavigationState>(
    declare_parameter<std::string>("topics.publishers.state"),
    rclcpp::QoS(1));
  position_raw_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    declare_parameter<std::string>("topics.publishers.position_raw", "/magicelectric0/nav/sample_and_hold/position_raw"),
    rclcpp::QoS(1));
  model_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    declare_parameter<std::string>("topics.publishers.model_velocity", "/magicelectric0/nav/sample_and_hold/model_velocity"),
    rclcpp::QoS(1));
  debug_pub2_ = create_publisher<std_msgs::msg::Float64>(
    declare_parameter<std::string>("topics.publishers.course_meas_debug2", "dummy2"),
    rclcpp::QoS(1));
  debug_pub1_ = create_publisher<std_msgs::msg::Float64>(
    declare_parameter<std::string>("topics.publishers.course_meas_debug1", "dummy1"),
    rclcpp::QoS(1));
}

/**
 * @brief Initialise Services
 */
void SampleAndHold::initialiseServices() {
  tune_position_ekf_srv_ = create_service<nav_filters::srv::TunePositionEkf>(
    declare_parameter<std::string>("topics.services.tune_position_ekf", "/magicelectric0/nav/sample_and_hold/tune_position_ekf"),
    [this](const std::shared_ptr<nav_filters::srv::TunePositionEkf::Request> request,
           std::shared_ptr<nav_filters::srv::TunePositionEkf::Response> response) {
      tune_position_ekf_callback(request, response);
    });
}

void SampleAndHold::tune_position_ekf_callback(
  const std::shared_ptr<nav_filters::srv::TunePositionEkf::Request> request,
  std::shared_ptr<nav_filters::srv::TunePositionEkf::Response> response) {
  if (request->process_noise_pos < 0.0 || request->process_noise_current < 0.0 ||
      request->measurement_noise_pos <= 0.0) {
    response->success = false;
    response->message = "Invalid EKF gains: require q>=0, r>0.";
    return;
  }

  ekf_q_pos_ = request->process_noise_pos;
  ekf_q_current_ = request->process_noise_current;
  ekf_r_pos_ = request->measurement_noise_pos;

  Q_pos_.setZero();
  Q_pos_.diagonal() << ekf_q_pos_, ekf_q_pos_, ekf_q_current_, ekf_q_current_;
  R_pos_.setIdentity();
  R_pos_ *= ekf_r_pos_;

  response->success = true;
  response->message = "Position EKF gains updated.";
}

/**
 * @brief Initialise Timers
 */
void SampleAndHold::initialiseTimers() {
  auto period = std::chrono::nanoseconds( static_cast<int64_t>(1e9 / node_frequency_));
  timer_ = create_timer(period, [this]() {timerCallback();});
  return;
}

void SampleAndHold::measurement_callback(farol2_interfaces::msg::Measurement::ConstSharedPtr msg) {
  // Update filter state depending on measurement type 
  switch(msg->type){
    /* Orientation: roll, pitch, yaw */
    case farol2_interfaces::msg::Measurement::MEAS_ATTITUDE:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation.x = farol2_utils::rad2deg(farol2_utils::wrapToPi(msg->value[0]));
      filter_state_msg_.orientation.y = farol2_utils::rad2deg(farol2_utils::wrapToPi(msg->value[1]));
      filter_state_msg_.orientation.z = farol2_utils::rad2deg(farol2_utils::wrapTo2Pi(msg->value[2]));
      break;
    /* Orientation rate: roll rate, pitch rate, yaw rate */
    case farol2_interfaces::msg::Measurement::MEAS_ANGULAR_VELOCITY:
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement ORIENTATION_RATE has incorrect length or type.");
        break;
      }
      filter_state_msg_.orientation_rate.x = farol2_utils::rad2deg(msg->value[0]);
      filter_state_msg_.orientation_rate.y = farol2_utils::rad2deg(msg->value[1]);
      filter_state_msg_.orientation_rate.z = farol2_utils::rad2deg(msg->value[2]);
      last_yaw_rate_meas_ = farol2_utils::rad2deg(msg->value[2]);
      break;
    /* UTM position (easting, northing) and UTM zone */
    case farol2_interfaces::msg::Measurement::MEAS_UTM_POSITION: {
      if (msg->value.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Measurement UTM_POSITION has incorrect length or type.");
        break;
      }

      geometry_msgs::msg::Vector3 raw_utm_msg;
      raw_utm_msg.x = msg->value[0];
      raw_utm_msg.y = msg->value[1];
      raw_utm_msg.z = msg->value[2];
      position_raw_pub_->publish(raw_utm_msg);

      latest_utm_zone_ = static_cast<int>(msg->value[2]);
      // std::cout << "Received GPS measurement: northing = " << msg->value[0] << ", easting = " << msg->value[1] << ", zone = " << msg->value[2] << std::endl;

      if (use_position_ekf_) {
        if (!position_ekf_initialized_) {
          // std::cout << "Initializing position EKF with first GPS measurement." << std::endl;
          x_pos_.setZero();
          x_pos_(0) = msg->value[0];
          x_pos_(1) = msg->value[1];
          P_pos_.setZero();
          P_pos_(0, 0) = ekf_p0_pos_;
          P_pos_(1, 1) = ekf_p0_pos_;
          P_pos_(2, 2) = ekf_p0_current_;
          P_pos_(3, 3) = ekf_p0_current_;
          position_ekf_initialized_ = true;
        } else {
          // std::cout << "Updating position EKF with GPS measurement." << std::endl;
          update_position_ekf(msg->value[0], msg->value[1]);
        }
        last_gps_update_time_s_ = clock_->now().seconds();
      } else {
        // std::cout << "Not using position EKF, directly setting position from GPS measurement." << std::endl;
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
      }

      break;
    }
    /* Depth */
    case farol2_interfaces::msg::Measurement::MEAS_DEPTH:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement DEPTH has incorrect length or type.");
        break;
      }
      filter_state_msg_.depth = msg->value[0];
      break;
    /* Altimeter */
    case farol2_interfaces::msg::Measurement::MEAS_ALTIMETER:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement ALTIMETER has incorrect length or type.");
        break;
      }
      filter_state_msg_.altimeter = msg->value[0];
      break;
    /* Altitude realtive to the ellipsoid, WGS84 */
    case farol2_interfaces::msg::Measurement::MEAS_ALTITUDE_WGS84:
      if (msg->value.size() != 1) {
        RCLCPP_ERROR(get_logger(), "Measurement ALTITUDE_WGS84 has incorrect length or type.");
        break;
      }
      filter_state_msg_.altitude_ellipsoidal = msg->value[0];
      break;
    /* Inertial velocity expressed in the body */
    case farol2_interfaces::msg::Measurement::MEAS_INERTIAL_VELOCITY: {
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
            Eigen::AngleAxisd(farol2_utils::deg2rad(filter_state_msg_.orientation.z),   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(farol2_utils::deg2rad(filter_state_msg_.orientation.y), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(farol2_utils::deg2rad(filter_state_msg_.orientation.x),  Eigen::Vector3d::UnitX())
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
    case farol2_interfaces::msg::Measurement::MEAS_FLUID_VELOCITY:
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
  double dt = 1.0 / node_frequency_;
  // Fill header 
  filter_state_msg_.header.stamp = clock_->now();

  if (use_position_ekf_ && position_ekf_initialized_) {
    const double now_s = clock_->now().seconds();
    if (last_gps_update_time_s_ >= 0.0 && (now_s - last_gps_update_time_s_) <= gps_timeout_s_) {
      predict_position_ekf(dt);
    }
    publish_position_from_ekf();
  }

  // Yaw-rate filtering executed in fixed-rate timer loop (not in measurement callback).
  if(use_yaw_rate_notch_filter_) {
    try {
      yaw_rate_notch_filter_.step(filter_state_msg_.orientation_rate.z, dt);
      filter_state_msg_.heading_rate = yaw_rate_notch_filter_.y();
      filter_state_msg_.heading_rate = smooth_deadzone(filter_state_msg_.heading_rate, 2.5, 4.0);
      // if(abs(filter_state_msg_.heading_rate<2.5)){
      //   filter_state_msg_.heading_rate = 0;
      // }
    } catch (const std::exception &e) {
      filter_state_msg_.heading_rate = filter_state_msg_.orientation_rate.z;
    }
  }
  if(use_yaw_rate_kf_){
    ///////////////////////////////////////////////////////
    //  Kalman filter to estimate yaw rate without waves //
    ///////////////////////////////////////////////////////

    // Update F and B matrices based on dt
    double m_r = 88.478840;  // Added mass in yaw
    double N_r= 14.334052;  // Damping in yaw
    double N_rr= 260.675031;  // Damping in yaw
    // double m_uv = -2.967651;
    A_ << 1, dt,
          0, 1 - (N_r / m_r) * dt;
    B_ << 0,
          (1.0 / m_r) * dt;
    // Compute torque from rudder angle
    double K_s_ = 1.0;
    double rudder_cm_distance_ = 4.0; 
    double fluid_velocity = sqrt(pow(filter_state_msg_.body_velocity_inertial.x, 2) + pow(filter_state_msg_.body_velocity_inertial.y, 2));
    double tau_r = rudder_angle_ * (K_s_ * rudder_cm_distance_ * fluid_velocity); 
    // double u = filter_state_msg_.body_velocity_inertial.x;
    // double v = filter_state_msg_.body_velocity_inertial.y;
    debug_pub2_->publish(std_msgs::msg::Float64().set__data(tau_r));


    // Kalman filter predict
    // x_ = A_ * x_ + B_ * tau_r;
    x_(0) += dt*x_(1);  
    x_(1) += dt*( - (N_r / m_r)*x_(1) - (N_rr / m_r)*x_(1)*abs(x_(1)) + (1.0 / m_r)*tau_r);
    P_ = A_ * P_ * A_.transpose() + Q_;

    // Kalman filter update (using yaw measurement)
    double yaw_meas_rad = farol2_utils::deg2rad(filter_state_msg_.orientation.z);
    Eigen::VectorXd z(1);
    z << yaw_meas_rad;
    Eigen::VectorXd y = (z - C_ * x_);
    y(0) = farol2_utils::wrapToPi(y(0));
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_;
    Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix2d::Identity() - K * C_) * P_;

    // Wrap yaw to [0, 2*pi)
    x_(0) = farol2_utils::wrapToPi(x_(0));
    debug_pub1_->publish(std_msgs::msg::Float64().set__data(farol2_utils::rad2deg(farol2_utils::wrapTo2Pi(x_(0)))));


    // Set filtered values
    filter_state_msg_.heading_rate = farol2_utils::rad2deg(x_(1));
  }
  if(use_course_cf_){
    ////////////////////////////////////////////////////////////////
    //  Estimate course angle using a simple complementary filter //
    ////////////////////////////////////////////////////////////////

    // Measurement: course angle from inertial velocity
    double course_angle_meas = farol2_utils::wrapTo2Pi(std::atan2(filter_state_msg_.ned_velocity_inertial.y, filter_state_msg_.ned_velocity_inertial.x));
    // debug_pub2_->publish(std_msgs::msg::Float64().set__data(farol2_utils::rad2deg(course_angle_meas)));
      
    // Predict with gyro integration
    course_angle_est_ += farol2_utils::deg2rad(filter_state_msg_.heading_rate) * dt;
    course_angle_est_ = farol2_utils::wrapTo2Pi(course_angle_est_);
    
    // Correct estimate
    course_angle_est_ += (1.0 - std::exp(-course_angle_cutoff_frequency_ * dt)) * farol2_utils::wrapToPi(course_angle_meas - course_angle_est_);
    course_angle_est_ = farol2_utils::wrapTo2Pi(course_angle_est_);

    // Output in degrees
    filter_state_msg_.course_angle = farol2_utils::rad2deg(course_angle_est_);
  }

  // Publish filter state message 
  state_pub_->publish(filter_state_msg_);
}

double SampleAndHold::rpm_to_body_speed_mps(double dt) {
  if (dt <= 0.0) {
    return u_estimated_;
  }

  const double now_s = clock_->now().seconds();
  const bool rpm_is_stale = (last_rpm_command_time_s_ < 0.0 || (now_s - last_rpm_command_time_s_) > rpm_timeout_s_);

  double rpm_cmd = rpm_is_stale ? 0.0 : latest_rpm_command_;
  rpm_cmd = std::clamp(rpm_cmd, rpm_min_, rpm_max_);

  if (!rpm_model_initialized_) {
    rpm_model_state_ = rpm_cmd;
    u_estimated_ = 0.0; // Start at rest
    rpm_model_initialized_ = true;
  }

  const double max_rpm_step = std::max(0.0, rpm_rate_limit_) * dt;
  rpm_model_state_ = std::clamp(rpm_cmd, rpm_model_state_ - max_rpm_step, rpm_model_state_ + max_rpm_step);

  // Check if RPM is within override zone (within 1% of override_rpms)
  const double rpm_threshold = std::abs(override_rpms_) * 0.01;
  const bool in_override_zone = (std::abs(rpm_model_state_ - override_rpms_) <= rpm_threshold);
  
  if (in_override_zone) {
    time_in_override_zone_s_ += dt;
  } else {
    time_in_override_zone_s_ = 0.0;
  }

  // Apply override velocity if RPM has been steady long enough
  if (override_timeout_s_ >= 0.0 && time_in_override_zone_s_ >= override_timeout_s_ && std::abs(override_velocity_) > 1e-6) {
    u_estimated_ = override_velocity_;
    return u_estimated_;
  }

  const double u = u_estimated_;
  const double rps = rpm_model_state_ / 60.0;
  double tau_u = 0.0;
  if (std::abs(rps * prop_pitch_) >= 1e-6) {
    tau_u = 2.0 * rho_ * rps * rps * std::pow(prop_diameter_, 4.0) * k_t_bp_ * (1.0 - u / (rps * prop_pitch_));
    if (rpm_model_state_ < 0.0) {
      tau_u = -tau_u;
    }
  }
  std::cout << "tau_u = " << tau_u << std::endl;

  const double m_u_safe = (std::abs(m_u_) < 1e-6) ? 1e-6 : m_u_;
  const double u_dot = (1.0 / m_u_safe) * (tau_u + x_u_ * u + x_uu_ * std::abs(u) * u);
  u_estimated_ += dt * u_dot;
  return u_estimated_;
}

void SampleAndHold::predict_position_ekf(double dt) {
  // std::cout << "EKF position predict: x_pos = [" << x_pos_.transpose() << "], P_pos = [" << P_pos_ << "]" << std::endl;
  const double psi = farol2_utils::deg2rad(filter_state_msg_.orientation.z);
  const double vm_body = rpm_to_body_speed_mps(dt);
  const double vn_m = vm_body * std::cos(psi);
  const double ve_m = vm_body * std::sin(psi);

  geometry_msgs::msg::Vector3 model_velocity_msg;
  model_velocity_msg.x = vn_m;
  model_velocity_msg.y = ve_m;
  model_velocity_msg.z = 0.0;
  model_velocity_pub_->publish(model_velocity_msg);

  x_pos_(0) += dt * (vn_m + x_pos_(2));
  x_pos_(1) += dt * (ve_m + x_pos_(3)); 

  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  Eigen::Matrix4d Qd = Q_pos_ * dt;
  P_pos_ = F * P_pos_ * F.transpose() + Qd;
}

void SampleAndHold::update_position_ekf(double northing, double easting) {
  Eigen::Vector2d z;
  z << northing, easting;

  const Eigen::Vector2d y = z - H_pos_ * x_pos_;
  // std::cout << "EKF position update: innovation y = [" << y.transpose() << "]" << std::endl;
  const Eigen::Matrix2d S = H_pos_ * P_pos_ * H_pos_.transpose() + R_pos_;
  const Eigen::Matrix<double, 4, 2> K = P_pos_ * H_pos_.transpose() * S.inverse();

  x_pos_ = x_pos_ + K * y;
  P_pos_ = (Eigen::Matrix4d::Identity() - K * H_pos_) * P_pos_;
}

void SampleAndHold::publish_position_from_ekf() {
  filter_state_msg_.utm_position.northing = x_pos_(0);
  filter_state_msg_.utm_position.easting = x_pos_(1);
  filter_state_msg_.utm_position.utm_zone = latest_utm_zone_;
  filter_state_msg_.current_velocity_inertial.x = x_pos_(2);
  filter_state_msg_.current_velocity_inertial.y = x_pos_(3);
  filter_state_msg_.current_velocity_inertial.z = 0.0;

  GeographicLib::UTMUPS::Reverse(
    latest_utm_zone_,
    true,
    x_pos_(1),
    x_pos_(0),
    filter_state_msg_.global_position.latitude,
    filter_state_msg_.global_position.longitude);
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
