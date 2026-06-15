#include <farol2_nav/filters/position_current_ekf.hpp>

#include <farol2_utils/angles.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace farol2_nav
{
namespace filters
{

void PositionCurrentEkfFilter::configure(rclcpp::Node & node)
{
  tune_ekf_srv_ = node.create_service<farol2_nav::srv::TunePositionEkf>(
    "position_current_ekf/tune",
    [this](
      const std::shared_ptr<farol2_nav::srv::TunePositionEkf::Request> req,
      std::shared_ptr<farol2_nav::srv::TunePositionEkf::Response> res)
    {
      this->on_tune_ekf(req, res);
    });

  q_pos_ = node.declare_parameter<double>("plugins.position_current_ekf.process_noise_pos", 0.1);
  q_current_ = node.declare_parameter<double>("plugins.position_current_ekf.process_noise_current", 0.01);
  r_pos_ = node.declare_parameter<double>("plugins.position_current_ekf.measurement_noise_pos", 1.0);
  p0_pos_ = node.declare_parameter<double>("plugins.position_current_ekf.init_cov_pos", 25.0);
  p0_current_ = node.declare_parameter<double>("plugins.position_current_ekf.init_cov_current", 1.0);
  init_current_x_ = node.declare_parameter<double>("plugins.position_current_ekf.init_current_x", 0.0);
  init_current_y_ = node.declare_parameter<double>("plugins.position_current_ekf.init_current_y", 0.0);
  override_position_state_ = node.declare_parameter<bool>("plugins.position_current_ekf.override_position_state", true);

  rpm_min_ = node.declare_parameter<double>("plugins.position_current_ekf.rpm_min", -2000.0);
  rpm_max_ = node.declare_parameter<double>("plugins.position_current_ekf.rpm_max", 2000.0);
  rpm_rate_limit_ = node.declare_parameter<double>("plugins.position_current_ekf.rpm_rate_limit", 200.0);
  rho_ = node.declare_parameter<double>("plugins.position_current_ekf.rho", 1025.0);
  prop_pitch_ = node.declare_parameter<double>("plugins.position_current_ekf.prop_pitch", 0.381);
  prop_diameter_ = node.declare_parameter<double>("plugins.position_current_ekf.prop_diameter", 0.4318);
  k_t_bp_ = node.declare_parameter<double>("plugins.position_current_ekf.k_t_bp", 0.061461915);
  m_u_ = node.declare_parameter<double>("plugins.position_current_ekf.m_u", 3100.0);
  if (m_u_ <= 0.0) {
    throw std::invalid_argument("plugins.position_current_ekf.m_u must be > 0");
  }
  m_uv_ = node.declare_parameter<double>("plugins.position_current_ekf.m_uv", -2.967651);
  m_v_ = m_u_ - m_uv_;
  if (m_v_ <= 0.0) {
    throw std::invalid_argument("plugins.position_current_ekf.m_v (computed as m_u - m_uv) must be > 0");
  }
  x_u_ = node.declare_parameter<double>("plugins.position_current_ekf.x_u", 0.0);
  x_uu_ = node.declare_parameter<double>("plugins.position_current_ekf.x_uu", -96.2270);
  Y_v_ = node.declare_parameter<double>("plugins.position_current_ekf.Y_v", -500.0);
  Y_vv_ = node.declare_parameter<double>("plugins.position_current_ekf.Y_vv", -1500.0);

  H_.setZero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  Q_.setZero();
  Q_.diagonal() << q_pos_, q_pos_, q_current_, q_current_;

  R_.setIdentity();
  R_ *= std::max(1e-9, r_pos_);
}

void PositionCurrentEkfFilter::on_tune_ekf(
  const std::shared_ptr<farol2_nav::srv::TunePositionEkf::Request> req,
  std::shared_ptr<farol2_nav::srv::TunePositionEkf::Response> res)
{
  if (req->process_noise_pos <= 0.0 ||
    req->process_noise_current <= 0.0 ||
    req->measurement_noise_pos <= 0.0)
  {
    res->success = false;
    res->message =
      "All EKF noise parameters must be > 0 (process_noise_pos, process_noise_current, measurement_noise_pos).";
    return;
  }

  q_pos_ = req->process_noise_pos;
  q_current_ = req->process_noise_current;
  r_pos_ = req->measurement_noise_pos;

  Q_.setZero();
  Q_.diagonal() << q_pos_, q_pos_, q_current_, q_current_;
  R_.setIdentity();
  R_ *= std::max(1e-9, r_pos_);

  res->success = true;
  res->message = "Position-current EKF noise parameters updated.";
}

Eigen::Vector2d PositionCurrentEkfFilter::rpm_to_body_velocity_mps(
  const MeasurementSnapshot & m, const State & s, double dt_s)
{
  if (dt_s <= 0.0) {
    return {u_estimated_, v_estimated_};
  }

  double rpm_cmd = 0.0;
  if (m.thruster_rpm != nullptr && !m.thruster_rpm->rpm.empty()) {
    rpm_cmd = m.thruster_rpm->rpm[0];
  }
  rpm_cmd = std::clamp(rpm_cmd, rpm_min_, rpm_max_);

  if (!rpm_model_initialized_) {
    rpm_model_state_ = rpm_cmd;
    u_estimated_ = 0.0;
    v_estimated_ = 0.0;
    rpm_model_initialized_ = true;
  }

  const double max_rpm_step = std::max(0.0, rpm_rate_limit_) * dt_s;
  rpm_model_state_ = std::clamp(rpm_cmd, rpm_model_state_ - max_rpm_step, rpm_model_state_ + max_rpm_step);

  const double u = u_estimated_;
  const double v = v_estimated_;
  const double rps = rpm_model_state_ / 60.0;
  double tau_u = 0.0;
  if (std::abs(rps * prop_pitch_) >= 1e-6) {
    tau_u = 2.0 * rho_ * rps * rps * std::pow(prop_diameter_, 4.0) *
      k_t_bp_ * (1.0 - u / (rps * prop_pitch_));
    if (rpm_model_state_ < 0.0) {
      tau_u = -tau_u;
    }
  }

  const double r = farol2_utils::deg2rad(s.angular_velocity(2));

  const double u_dot = (1.0 / m_u_) * (tau_u + m_v_* v * r + x_u_ * u + x_uu_ * std::abs(u) * u);
  const double v_dot = (1.0 / m_v_) * (0.0   - m_u_* u * r + Y_v_ * v + Y_vv_ * std::abs(v) * v);

  u_estimated_ += dt_s * u_dot;
  v_estimated_ += dt_s * v_dot;

  return {u_estimated_, v_estimated_};
}

void PositionCurrentEkfFilter::compute(double dt_s, const MeasurementSnapshot & m, State & s)
{
  // Measurement is taken from the pass-through state (already populated from GNSS/UTM inputs).
  // This keeps one canonical measurement path in the pipeline instead of each filter
  // reading raw topics independently.
  const bool has_position_measurement = (m.gnss != nullptr) || (m.utm_ned != nullptr);
  const double measured_x_m = s.northing;
  const double measured_y_m = s.easting;

  // initialize ekf whith the first measurement
  if (!initialized_) {
    if (!has_position_measurement) {
      return;
    }

    x_.setZero();
    x_(0) = measured_x_m;
    x_(1) = measured_y_m;
    x_(2) = init_current_x_;
    x_(3) = init_current_y_;

    P_.setZero();
    P_(0, 0) = p0_pos_;
    P_(1, 1) = p0_pos_;
    P_(2, 2) = p0_current_;
    P_(3, 3) = p0_current_;

    initialized_ = true;
  }

  const double dt = std::max(0.0, dt_s);

  const double psi = std::atan2(s.rotation_bn(1, 0), s.rotation_bn(0, 0));
  const Eigen::Vector2d vm_body = rpm_to_body_velocity_mps(m, s, dt);
  const double u_model = vm_body(0);
  const double v_model = vm_body(1);
  const double vx_model = u_model * std::cos(psi) - v_model * std::sin(psi);
  const double vy_model = u_model * std::sin(psi) + v_model * std::cos(psi);

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
  // ---------------------------
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
    s.northing = x_(0);
    s.easting = x_(1);
  }
  s.current_velocity_ned(0) = x_(2);
  s.current_velocity_ned(1) = x_(3);
  s.current_velocity_ned(2) = 0.0;
}

}  // namespace filters
}  // namespace farol2_nav
