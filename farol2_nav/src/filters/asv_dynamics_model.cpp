#include <farol2_nav/filters/asv_dynamics_model.hpp>

#include <farol2_utils/angles.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace farol2_nav
{
namespace filters
{

void AsvDynamicsModelFilter::configure(rclcpp::Node & node)
{
  uvr_pub_ = node.create_publisher<geometry_msgs::msg::Vector3>(
    "asv_dynamics_model/uvr", rclcpp::QoS(10));
  tau_pub_ = node.create_publisher<geometry_msgs::msg::Vector3>(
    "asv_dynamics_model/tau", rclcpp::QoS(10));

  rpm_min_ = node.declare_parameter<double>("plugins.asv_dynamics_model.rpm_min", -2000.0);
  rpm_max_ = node.declare_parameter<double>("plugins.asv_dynamics_model.rpm_max", 2000.0);
  rpm_rate_limit_ = node.declare_parameter<double>("plugins.asv_dynamics_model.rpm_rate_limit", 200.0);
  rho_ = node.declare_parameter<double>("plugins.asv_dynamics_model.rho", 1025.0);
  prop_pitch_ = node.declare_parameter<double>("plugins.asv_dynamics_model.prop_pitch", 0.381);
  prop_diameter_ = node.declare_parameter<double>("plugins.asv_dynamics_model.prop_diameter", 0.4318);
  k_t_bp_ = node.declare_parameter<double>("plugins.asv_dynamics_model.k_t_bp", 0.061461915);
  m_u_ = node.declare_parameter<double>("plugins.asv_dynamics_model.m_u", 3100.0);
  if (m_u_ <= 0.0) {
    throw std::invalid_argument("plugins.asv_dynamics_model.m_u must be > 0");
  }
  m_uv_ = node.declare_parameter<double>("plugins.asv_dynamics_model.m_uv", -2.967651);
  m_v_ = m_u_ - m_uv_;
  if (m_v_ <= 0.0) {
    throw std::invalid_argument("plugins.asv_dynamics_model.m_v (computed as m_u - m_uv) must be > 0");
  }
  x_u_ = node.declare_parameter<double>("plugins.asv_dynamics_model.x_u", 0.0);
  x_uu_ = node.declare_parameter<double>("plugins.asv_dynamics_model.x_uu", -96.2270);
  Y_v_ = node.declare_parameter<double>("plugins.asv_dynamics_model.Y_v", -500.0);
  Y_vv_ = node.declare_parameter<double>("plugins.asv_dynamics_model.Y_vv", -1500.0);

  m_r_ = node.declare_parameter<double>("plugins.asv_dynamics_model.m_r", 88.478840);
  if (std::abs(m_r_) < 1e-9) {
    throw std::invalid_argument("plugins.asv_dynamics_model.m_r must satisfy |m_r| >= 1e-9");
  }
  damping_linear_ = node.declare_parameter<double>("plugins.asv_dynamics_model.damping", 14.334052);
  damping_quadratic_ =
    node.declare_parameter<double>("plugins.asv_dynamics_model.damping_quadratic", 260.675031);
  rudder_cm_distance_ = node.declare_parameter<double>("plugins.asv_dynamics_model.rudder_arm", 4.0);
  const double rudder_angle_limit_deg =
    node.declare_parameter<double>("plugins.asv_dynamics_model.rudder_angle_limit_deg", 36.0);
  rudder_angle_limit_rad_ = farol2_utils::deg2rad(std::abs(rudder_angle_limit_deg));
  K_L_ = node.declare_parameter<double>("plugins.asv_dynamics_model.K_L", 1.398093);
  K_D0_ = node.declare_parameter<double>("plugins.asv_dynamics_model.K_D0", 0.000575);
  K_D1_ = node.declare_parameter<double>("plugins.asv_dynamics_model.K_D1", 0.520834);
  torque_bias_ = node.declare_parameter<double>("plugins.asv_dynamics_model.torque_bias", 0.0);

  init_u_ = node.declare_parameter<double>("plugins.asv_dynamics_model.init_u", 0.0);
  init_v_ = node.declare_parameter<double>("plugins.asv_dynamics_model.init_v", 0.0);
  init_r_ = node.declare_parameter<double>("plugins.asv_dynamics_model.init_r", 0.0);

  initialized_ = false;
  rpm_model_state_ = 0.0;
}

double AsvDynamicsModelFilter::get_tau_r(double rudder_angle_rad, double u, double v, double r) const
{
  const double delta_rud =
    std::clamp(rudder_angle_rad, -rudder_angle_limit_rad_, rudder_angle_limit_rad_);

  Eigen::Vector2d V_s;
  V_s(0) = u;
  V_s(1) = v - r * rudder_cm_distance_;

  const double gamma = farol2_utils::wrapToPi(std::atan2(V_s(1), V_s(0)));
  const double alpha = farol2_utils::wrapToPi(delta_rud + gamma);
  const double V_sq = V_s.squaredNorm();

  const double lift = K_L_ * alpha * V_sq;
  const double drag = (K_D0_ + K_D1_ * alpha * alpha) * V_sq;

  return rudder_cm_distance_ * (lift * std::cos(gamma) + drag * std::sin(gamma));
}

void AsvDynamicsModelFilter::compute(double dt_s, const MeasurementSnapshot & m, State &)
{
  const double dt = std::max(0.0, dt_s);
  if (dt <= 0.0) {
    return;
  }

  double rpm_cmd = 0.0;
  if (m.rpm_command != nullptr && !m.rpm_command->rpm.empty()) {
    rpm_cmd = m.rpm_command->rpm[0];
  }
  rpm_cmd = std::clamp(rpm_cmd, rpm_min_, rpm_max_);

  if (!initialized_) {
    rpm_model_state_ = rpm_cmd;
    u_model_ = init_u_;
    v_model_ = init_v_;
    r_model_ = init_r_;
    initialized_ = true;
  }

  const double max_rpm_step = std::max(0.0, rpm_rate_limit_) * dt;
  rpm_model_state_ = std::clamp(rpm_cmd, rpm_model_state_ - max_rpm_step, rpm_model_state_ + max_rpm_step);

  const double rps = rpm_model_state_ / 60.0;
  double tau_u = 0.0;
  if (std::abs(rps * prop_pitch_) >= 1e-6) {
    tau_u = 2.0 * rho_ * rps * rps * std::pow(prop_diameter_, 4.0) *
      k_t_bp_ * (1.0 - u_model_ / (rps * prop_pitch_));
    if (rpm_model_state_ < 0.0) {
      tau_u = -tau_u;
    }
  }

  double rudder_angle_rad = 0.0;
  if (m.rudder_angle != nullptr) {
    rudder_angle_rad = farol2_utils::deg2rad(m.rudder_angle->data);
  }
  const double tau_r = get_tau_r(rudder_angle_rad, u_model_, v_model_, r_model_);

  const double u_dot =
    (1.0 / m_u_) * (tau_u + m_v_ * v_model_ * r_model_ + x_u_ * u_model_ + x_uu_ * std::abs(u_model_) * u_model_);
  const double v_dot =
    (1.0 / m_v_) * (-m_u_ * u_model_ * r_model_ + Y_v_ * v_model_ + Y_vv_ * std::abs(v_model_) * v_model_);
  const double r_abs = std::abs(r_model_);
  const double r_dot =
    (tau_r + torque_bias_ + m_uv_ * u_model_ * v_model_ - damping_linear_ * r_model_ -
    damping_quadratic_ * r_model_ * r_abs) / m_r_;

  u_model_ += dt * u_dot;
  v_model_ += dt * v_dot;
  r_model_ += dt * r_dot;

  uvr_msg_.x = u_model_;
  uvr_msg_.y = v_model_;
  uvr_msg_.z = r_model_;
  uvr_pub_->publish(uvr_msg_);

  tau_msg_.x = tau_u;
  tau_msg_.y = tau_r;
  tau_msg_.z = 0.0;
  tau_pub_->publish(tau_msg_);
}

}  // namespace filters
}  // namespace farol2_nav
