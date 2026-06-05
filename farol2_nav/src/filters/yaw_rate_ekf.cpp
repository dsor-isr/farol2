#include <farol2_nav/filters/yaw_rate_ekf.hpp>

#include <farol2_utils/angles.hpp>

#include <algorithm>
#include <cmath>

namespace farol2_nav
{
namespace filters
{

void YawRateEkfFilter::configure(rclcpp::Node & node)
{
  // Configure torque bias publisher for external monitoring.
  torque_bias_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/torque_bias", rclcpp::QoS(10));
  yaw_rate_filtered_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/yaw_rate_filtered", rclcpp::QoS(10));
  current_yaw_rate_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/current_yaw_rate", rclcpp::QoS(10));
  innovation_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/innovation", rclcpp::QoS(10));
  delayed_yaw_rate_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/delayed_yaw_rate", rclcpp::QoS(10));
  tau_r_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/tau_r", rclcpp::QoS(10));
  torque_gain_pub_ = node.create_publisher<std_msgs::msg::Float32>("yaw_rate_ekf/torque_gain", rclcpp::QoS(10));
  tune_ekf_srv_ = node.create_service<farol2_nav::srv::TuneYawRateEkf>(
    "yaw_rate_ekf/tune",
    [this](
      const std::shared_ptr<farol2_nav::srv::TuneYawRateEkf::Request> req,
      std::shared_ptr<farol2_nav::srv::TuneYawRateEkf::Response> res)
    {
      this->on_tune_ekf(req, res);
    });

  // EKF parameters
  q_r_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.process_noise_yaw_rate", 0.02);
  q_b_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.process_noise_bias", 10.0);
  q_g_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.process_noise_gain", 1e-3);
  r_meas_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.measurement_noise_yaw_rate", 0.1);
  p0_r_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.init_cov_yaw_rate", 1.0);
  p0_b_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.init_cov_bias", 1.0);
  p0_g_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.init_cov_gain", 1.0);

  // parameter for yaw_rate dynamical model
  m_r_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.inertia", 88.478840);
  if (std::abs(m_r_) < 1e-9) 
    throw std::invalid_argument("plugins.yaw_rate_ekf.inertia must satisfy |inertia| >= 1e-9");
  m_uv_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.m_uv", -2.967651);
  damping_linear_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.damping", 14.334052);
  damping_quadratic_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.damping_quadratic", 260.675031);

  // parameters for the rudder_angle to torque model
  Ks_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.torque_gain", 1.0);
  rudder_cm_distance_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.rudder_arm", 4.0);
  K_L_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.K_L", 1.398093);
  K_D0_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.K_D0", 0.000575);
  K_D1_ = node.declare_parameter<double>("plugins.yaw_rate_ekf.K_D1", 0.520834);

  // parameters for the moving average prefilter on the IMU yaw-rate measurements.
  const int64_t measurement_window_samples = node.declare_parameter<int64_t>("plugins.yaw_rate_ekf.measurement_window_samples", 50);
  if (measurement_window_samples < 1) 
    throw std::invalid_argument("plugins.yaw_rate_ekf.measurement_window_samples must be >= 1");
  measurement_window_samples_ = static_cast<std::size_t>(measurement_window_samples);
  measurement_delay_samples_ = (measurement_window_samples_ - 1U) / 2U;
  history_max_samples_ = std::max<std::size_t>(2U, measurement_delay_samples_ + 1U); // Keep exactly the lag horizon plus the current step so delayed update index exists.

  // Yaw-rate is linear (no wrap). Sample-window MAF smooths IMU noise before EKF correction.
  yaw_rate_maf_.configure(measurement_window_samples_, false, false, 0.0);
    
  // Initialize state and covariance.
  P_.setZero();
  P_(0, 0) = std::max(1e-9, p0_r_);
  P_(1, 1) = std::max(1e-9, p0_b_);
  P_(2, 2) = std::max(1e-9, p0_g_);
  initialized_ = false;
  history_.clear();
}

void YawRateEkfFilter::on_tune_ekf(
  const std::shared_ptr<farol2_nav::srv::TuneYawRateEkf::Request> req,
  std::shared_ptr<farol2_nav::srv::TuneYawRateEkf::Response> res)
{
  if (req->process_noise_yaw_rate <= 0.0 ||
    req->process_noise_bias <= 0.0 ||
    req->measurement_noise_yaw_rate <= 0.0)
  {
    res->success = false;
    res->message =
      "All EKF noise parameters must be > 0 (process_noise_yaw_rate, process_noise_bias, measurement_noise_yaw_rate).";
    return;
  }

  q_r_ = req->process_noise_yaw_rate;
  q_b_ = req->process_noise_bias;
  r_meas_ = req->measurement_noise_yaw_rate;

  res->success = true;
  res->message = "Yaw-rate EKF noise parameters updated.";
}

double YawRateEkfFilter::get_torque(
  double rudder_angle_rad,
  const Eigen::Vector3d & velocity_through_water_body,
  double r) const
{
  const double u = velocity_through_water_body(0);
  const double v = velocity_through_water_body(1);
  

  Eigen::Vector2d V_s;
  V_s(0) = u;
  V_s(1) = v - r * rudder_cm_distance_;

  const double gamma = farol2_utils::wrapToPi(std::atan2(V_s(1), V_s(0)));
  const double alpha = farol2_utils::wrapToPi(rudder_angle_rad + gamma);
  const double V_sq = V_s.squaredNorm();

  const double lift = K_L_ * alpha * V_sq;
  const double drag = (K_D0_ + K_D1_ * alpha * alpha) * V_sq;

  return rudder_cm_distance_ * (lift * std::cos(gamma) + drag * std::sin(gamma));
}

void YawRateEkfFilter::predict(
  Eigen::Vector3d & x, Eigen::Matrix3d & P, double dt_s, double tau_r, const State & s) const
{
  const double dt = std::max(0.0, dt_s);
  if (dt <= 0.0) {
    return;
  }

  // Nonlinear model:
  // r_dot = (g*tau_r + b + m_uv*u*v - d1*r - d2*r*|r|) / I
  // b_dot = 0
  // g_dot = 0
  const double r = x(0);
  const double b = x(1);
  const double g = x(2);
  const double r_abs = std::abs(r);
  const double u = s.velocity_through_water_body(0);
  const double v = s.velocity_through_water_body(1);
  const double r_dot = (g * tau_r + b + m_uv_ * u * v - damping_linear_ * r - damping_quadratic_ * r * r_abs) / m_r_;

  x(0) = r + dt * r_dot;
  x(1) = b;
  x(2) = g;

  // Jacobian of the discrete dynamics for EKF covariance prediction.
  // d(r*|r|)/dr = 2|r| for r != 0; using 2|r| also at r=0 is acceptable.
  const double df_dr = (-damping_linear_ - 2.0 * damping_quadratic_ * r_abs) / m_r_;
  const double df_db = 1.0 / m_r_;
  const double df_dg = tau_r / m_r_;

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 0) += dt * df_dr;
  F(0, 1) += dt * df_db;
  F(0, 2) += dt * df_dg;

  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(0, 0) = std::max(1e-12, q_r_) * dt;
  Q(1, 1) = std::max(1e-12, q_b_) * dt;
  Q(2, 2) = std::max(1e-12, q_g_) * dt;
  P = F * P * F.transpose() + Q;
}

void YawRateEkfFilter::update(Eigen::Vector3d & x, Eigen::Matrix3d & P, double z_r) const
{
  Eigen::RowVector3d H;
  H << 1.0, 0.0, 0.0;

  const double y = z_r - (H * x)(0);
  std_msgs::msg::Float32 innovation_msg;
  innovation_msg.data = static_cast<float>(y);
  innovation_pub_->publish(innovation_msg);
  const double S_cov = (H * P * H.transpose())(0, 0) + std::max(1e-12, r_meas_);
  const Eigen::Vector3d K = P * H.transpose() / S_cov;

  x += K * y;

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  P = (I - K * H) * P;
}

void YawRateEkfFilter::compute(double dt_s, const MeasurementSnapshot & m, State & s)
{
  const double dt = std::max(0.0, dt_s);
  if (dt <= 0.0) {
    return;
  }

  // Measurement: IMU yaw-rate (rad/s).
  const bool has_imu = (m.imu != nullptr);
  double z_r = 0.0;
  if (has_imu) {
    yaw_rate_maf_.step(m.imu->angular_velocity.z);
    z_r = yaw_rate_maf_.y();
    yaw_rate_filtered_msg_.data = static_cast<float>(z_r);
    yaw_rate_filtered_pub_->publish(yaw_rate_filtered_msg_);
  }

  // Input torque from rudder command + fluid velocity estimate.
  double rudder_angle = 0.0;
  if (m.rudder_angle != nullptr) {
    rudder_angle = farol2_utils::deg2rad(m.rudder_angle->data);
  }
  const double tau_r = get_torque(rudder_angle, s.velocity_through_water_body, x_(0));
  tau_r_msg_.data = static_cast<float>(tau_r);
  tau_r_pub_->publish(tau_r_msg_);

  if (!initialized_) {
    x_.setZero();
    x_(0) = has_imu ? z_r : farol2_utils::deg2rad(s.angular_velocity(2));
    x_(1) = 0.0;
    x_(2) = 1.0;
    initialized_ = true;
    history_.clear();
  }

  // 1) Predict current step and append to history.
  Eigen::Vector3d x_pred = x_;
  Eigen::Matrix3d P_pred = P_;
  predict(x_pred, P_pred, dt, tau_r, s);
  history_.push_back(HistoryEntry{x_pred, P_pred, dt, tau_r});
  while (history_.size() > history_max_samples_) {
    history_.pop_front();
  }
  delayed_yaw_rate_msg_.data = static_cast<float>(history_.front().x_post(0));
  delayed_yaw_rate_pub_->publish(delayed_yaw_rate_msg_);
  current_yaw_rate_msg_.data = static_cast<float>(history_.back().x_post(0));
  current_yaw_rate_pub_->publish(current_yaw_rate_msg_);

  // 2) Apply correction at the oldest buffered state and re-roll forward.
  // With history size tied to delay+1, the oldest entry is the delayed state.
  if (has_imu && !history_.empty()) {
    update(history_.front().x_post, history_.front().P_post, z_r);

    for (std::size_t i = 1U; i < history_.size(); ++i) {
      Eigen::Vector3d x_reprop = history_[i - 1U].x_post;
      Eigen::Matrix3d P_reprop = history_[i - 1U].P_post;
      predict(x_reprop, P_reprop, history_[i].dt_s, history_[i].tau_r, s);
      history_[i].x_post = x_reprop;
      history_[i].P_post = P_reprop;
    }
  }

  x_ = history_.back().x_post;
  P_ = history_.back().P_post;

  // This module only outputs yaw-rate; all other state channels are untouched.
  s.angular_velocity(2) = farol2_utils::rad2deg(x_(0));
  torque_bias_msg_.data = static_cast<float>(x_(1));
  torque_bias_pub_->publish(torque_bias_msg_);
  torque_gain_msg_.data = static_cast<float>(x_(2));
  torque_gain_pub_->publish(torque_gain_msg_);
}

}  // namespace filters
}  // namespace farol2_nav
