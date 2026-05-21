#include <farol2_nav/filters/position_current_ekf.hpp>

#include <farol2_utils/angles.hpp>

#include <algorithm>
#include <cmath>

namespace farol2_nav
{
namespace filters
{

void PositionCurrentEkfFilter::configure(rclcpp::Node & node)
{
  q_pos_ = node.declare_parameter<double>("plugins.position_current_ekf.process_noise_pos", 0.1);
  q_current_ = node.declare_parameter<double>("plugins.position_current_ekf.process_noise_current", 0.01);
  r_pos_ = node.declare_parameter<double>("plugins.position_current_ekf.measurement_noise_pos", 1.0);
  p0_pos_ = node.declare_parameter<double>("plugins.position_current_ekf.init_cov_pos", 25.0);
  p0_current_ = node.declare_parameter<double>("plugins.position_current_ekf.init_cov_current", 1.0);
  override_position_state_ = node.declare_parameter<bool>("plugins.position_current_ekf.override_position_state", true);

  rpm_min_ = node.declare_parameter<double>("plugins.position_current_ekf.rpm_min", -2000.0);
  rpm_max_ = node.declare_parameter<double>("plugins.position_current_ekf.rpm_max", 2000.0);
  rpm_rate_limit_ = node.declare_parameter<double>("plugins.position_current_ekf.rpm_rate_limit", 200.0);
  rho_ = node.declare_parameter<double>("plugins.position_current_ekf.rho", 1025.0);
  prop_pitch_ = node.declare_parameter<double>("plugins.position_current_ekf.prop_pitch", 0.381);
  prop_diameter_ = node.declare_parameter<double>("plugins.position_current_ekf.prop_diameter", 0.4318);
  k_t_bp_ = node.declare_parameter<double>("plugins.position_current_ekf.k_t_bp", 0.061461915);
  m_u_ = node.declare_parameter<double>("plugins.position_current_ekf.m_u", 3100.0);
  x_u_ = node.declare_parameter<double>("plugins.position_current_ekf.x_u", 0.0);
  x_uu_ = node.declare_parameter<double>("plugins.position_current_ekf.x_uu", -96.2270);

  override_velocity_ = node.declare_parameter<double>("plugins.position_current_ekf.override_velocity", 0.0);
  override_rpms_ = node.declare_parameter<double>("plugins.position_current_ekf.override_rpms", 600.0);
  override_timeout_s_ = node.declare_parameter<double>("plugins.position_current_ekf.override_timeout_s", 5.0);

  H_.setZero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  Q_.setZero();
  Q_.diagonal() << q_pos_, q_pos_, q_current_, q_current_;

  R_.setIdentity();
  R_ *= std::max(1e-9, r_pos_);
  RCLCPP_INFO_STREAM(node.get_logger(), "PositionCurrentEkfFilter configured with Q diagonal [" << q_pos_ << ", " << q_pos_ << ", " << q_current_ << ", " << q_current_ << "] and R [" << r_pos_ << "]");
}

double PositionCurrentEkfFilter::rpm_to_body_speed_mps(const MeasurementSnapshot & m, double dt_s)
{
  if (dt_s <= 0.0) {
    return u_estimated_;
  }

  if (m.rpm_command == nullptr || m.rpm_command->rpm.empty()) {
    // If RPM is stale, force command to zero and reset override timer.
    time_in_override_zone_s_ = 0.0;
  }

  double rpm_cmd = 0.0;
  if (m.rpm_command != nullptr && !m.rpm_command->rpm.empty()) {
    rpm_cmd = m.rpm_command->rpm[0];
  }
  rpm_cmd = std::clamp(rpm_cmd, rpm_min_, rpm_max_);

  if (!rpm_model_initialized_) {
    rpm_model_state_ = rpm_cmd;
    u_estimated_ = 0.0;
    rpm_model_initialized_ = true;
  }

  const double max_rpm_step = std::max(0.0, rpm_rate_limit_) * dt_s;
  rpm_model_state_ = std::clamp(rpm_cmd, rpm_model_state_ - max_rpm_step, rpm_model_state_ + max_rpm_step);

  const double rpm_threshold = std::abs(override_rpms_) * 0.01;
  const bool in_override_zone = (std::abs(rpm_model_state_ - override_rpms_) <= rpm_threshold);

  if (m.rpm_command == nullptr || m.rpm_command->rpm.empty()) {
    time_in_override_zone_s_ = 0.0;
  } else if (in_override_zone) {
    time_in_override_zone_s_ += dt_s;
  } else {
    time_in_override_zone_s_ = 0.0;
  }

  if (override_timeout_s_ >= 0.0 && time_in_override_zone_s_ >= override_timeout_s_ &&
    std::abs(override_velocity_) > 1e-6)
  {
    u_estimated_ = override_velocity_;
    return u_estimated_;
  }

  const double u = u_estimated_;
  const double rps = rpm_model_state_ / 60.0;
  double tau_u = 0.0;
  if (std::abs(rps * prop_pitch_) >= 1e-6) {
    tau_u = 2.0 * rho_ * rps * rps * std::pow(prop_diameter_, 4.0) *
      k_t_bp_ * (1.0 - u / (rps * prop_pitch_));
    if (rpm_model_state_ < 0.0) {
      tau_u = -tau_u;
    }
  }

  const double m_u_safe = (std::abs(m_u_) < 1e-6) ? 1e-6 : m_u_;
  const double u_dot = (1.0 / m_u_safe) * (tau_u + x_u_ * u + x_uu_ * std::abs(u) * u);
  u_estimated_ += dt_s * u_dot;

  return u_estimated_;
}

void PositionCurrentEkfFilter::compute(double dt_s, const MeasurementSnapshot & m, State & s)
{
  // Measurement is taken from the pass-through state (already populated from GNSS/UTM inputs).
  // This keeps one canonical measurement path in the pipeline instead of each filter
  // reading raw topics independently.
  const bool has_position_measurement = (m.gnss != nullptr) || (m.utm_ned != nullptr);
  const double measured_x_m = s.northing_m;
  const double measured_y_m = s.easting_m;

  // EKF bootstrapping: initialize when the first valid position measurement appears.
  if (!initialized_) {
    if (!has_position_measurement) {
      return;
    }

    x_.setZero();
    x_(0) = measured_x_m;
    x_(1) = measured_y_m;

    P_.setZero();
    P_(0, 0) = p0_pos_;
    P_(1, 1) = p0_pos_;
    P_(2, 2) = p0_current_;
    P_(3, 3) = p0_current_;

    initialized_ = true;
  }

  const double dt = std::max(0.0, dt_s);

  // ---------------------------
  // EKF prediction step (model)
  // ---------------------------
  // State definition:
  // x = [x_pos, y_pos, current_x, current_y]^T
  //
  // Kinematics model:
  //   x_dot = v_x_model + current_x
  //   y_dot = v_y_model + current_y
  // where (v_x_model, v_y_model) comes from RPM-based surge estimate projected with yaw.
  //
  // Covariance model:
  //   P_k|k-1 = F P_k-1|k-1 F^T + Q_d
  const double psi = farol2_utils::deg2rad(s.attitude_deg(2));
  const double vm_body = rpm_to_body_speed_mps(m, dt);
  const double vx_model = vm_body * std::cos(psi);
  const double vy_model = vm_body * std::sin(psi);

  x_(0) += dt * (vx_model + x_(2));
  x_(1) += dt * (vy_model + x_(3));

  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  const Eigen::Matrix4d Qd = Q_ * dt;
  P_ = F * P_ * F.transpose() + Qd;

  // -------------------------------
  // EKF correction step (measurement)
  // -------------------------------
  // We only update when a fresh position measurement exists this cycle.
  // Measurement model:
  //   z = H x + v,  with H selecting [x_pos, y_pos]
  if (has_position_measurement) {
    Eigen::Vector2d z;
    z << measured_x_m, measured_y_m;

    const Eigen::Vector2d y = z - H_ * x_;
    const Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    const Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

    x_ += K * y;
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
  }

  // Write estimator output back to the shared pipeline state.
  if (override_position_state_) {
    s.northing_m = x_(0);
    s.easting_m = x_(1);
  }
  s.current_velocity_inertial_mps(0) = x_(2);
  s.current_velocity_inertial_mps(1) = x_(3);
  s.current_velocity_inertial_mps(2) = 0.0;
}

}  // namespace filters
}  // namespace farol2_nav
