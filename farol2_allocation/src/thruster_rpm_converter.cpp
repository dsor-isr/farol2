#include <thruster_rpm_converter.hpp>

#include <algorithm>
#include <cmath>

ThrusterRpmConverter ThrusterRpmConverter::staticCurve(
  std::vector<double> coef_fwd,
  std::vector<double> coef_bwd,
  double max_rpm,
  double min_rpm)
{
  ThrusterRpmConverter converter;
  converter.mode_ = Mode::STATIC_CURVE;
  converter.coef_fwd_ = std::move(coef_fwd);
  converter.coef_bwd_ = std::move(coef_bwd);
  converter.max_rpm_ = max_rpm;
  converter.min_rpm_ = min_rpm;
  return converter;
}

ThrusterRpmConverter ThrusterRpmConverter::thrusterRudder(
  int mode,
  std::vector<double> coef_fwd,
  std::vector<double> coef_bwd,
  double rho,
  double k_t_bp,
  double prop_pitch,
  double diameter,
  double max_rpm,
  double min_rpm)
{
  ThrusterRpmConverter converter;
  converter.mode_ = Mode::THRUSTER_RUDDER;
  converter.thruster_rudder_mode_ = mode;
  converter.coef_fwd_ = std::move(coef_fwd);
  converter.coef_bwd_ = std::move(coef_bwd);
  converter.rho_ = rho;
  converter.k_t_bp_ = k_t_bp;
  converter.prop_pitch_ = prop_pitch;
  converter.diameter_ = diameter;
  converter.max_rpm_ = max_rpm;
  converter.min_rpm_ = min_rpm;
  return converter;
}

void ThrusterRpmConverter::setSurge(double surge)
{
  surge_ = surge;
}

farol2_interfaces::msg::ThrusterRPM ThrusterRpmConverter::convert(
  const std::vector<double> & forces,
  const rclcpp::Time & stamp) const
{
  farol2_interfaces::msg::ThrusterRPM rpm_msg;
  rpm_msg.header.stamp = stamp;
  rpm_msg.rpm.reserve(forces.size());

  for (const double force : forces) {
    if (mode_ == Mode::STATIC_CURVE) {
      rpm_msg.rpm.push_back(forceToRpmStatic(force));
    } else {
      rpm_msg.rpm.push_back(forceToRpmThrusterRudder(force));
    }
  }

  return rpm_msg;
}

double ThrusterRpmConverter::forceToRpmStatic(double force) const
{
  if (force == 0.0) {
    return 0.0;
  }

  if (force > 0.0) {
    const double discriminant = coef_fwd_[1] * coef_fwd_[1] - 4.0 * coef_fwd_[0] * (coef_fwd_[2] - force);
    const double rpm_value = (-coef_fwd_[1] + std::sqrt(discriminant)) / (2.0 * coef_fwd_[0]);
    return std::min(rpm_value, max_rpm_);
  }

  const double discriminant = coef_bwd_[1] * coef_bwd_[1] - 4.0 * coef_bwd_[0] * (coef_bwd_[2] - force);
  const double rpm_value = (-coef_bwd_[1] + std::sqrt(discriminant)) / (2.0 * coef_bwd_[0]);
  return std::max(rpm_value, min_rpm_);
}

double ThrusterRpmConverter::forceToRpmThrusterRudder(double force) const
{
  if (thruster_rudder_mode_ == 0) {
    if (force == 0.0) {
      return 0.0;
    }

    const double a = rho_ * std::pow(diameter_, 4) * k_t_bp_;
    const double b = -rho_ * std::pow(diameter_, 4) * k_t_bp_ / prop_pitch_ * surge_;
    const double c = -force;

    if (c > 0.0) {
      double rpm_value = ((-b + std::sqrt(std::pow(b, 2) + 4.0 * a * c)) / (2.0 * a)) * 60.0;
      rpm_value = -rpm_value;
      return std::max(rpm_value, min_rpm_);
    }

    const double rpm_value = ((-b + std::sqrt(std::pow(b, 2) - 4.0 * a * c)) / (2.0 * a)) * 60.0;
    return std::min(rpm_value, max_rpm_);
  }

  return forceToRpmStatic(force);
}