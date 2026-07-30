#include <rudder_allocator.hpp>

#include <algorithm>
#include <cmath>

#include <farol2_utils/angles.hpp>

RudderAllocator::RudderAllocator(
  double rudder_angle_min_rad,
  double rudder_angle_max_rad,
  double rudder_cm_distance,
  double k_s,
  double k_l,
  double k_d0,
  double k_d1)
: rudder_angle_min_(rudder_angle_min_rad),
  rudder_angle_max_(rudder_angle_max_rad),
  rudder_cm_distance_(rudder_cm_distance),
  k_s_(k_s),
  k_l_(k_l),
  k_d0_(k_d0),
  k_d1_(k_d1)
{}

RudderAllocationResult RudderAllocator::compute(
  const farol2_interfaces::msg::NavigationState & nav_state,
  double tau_r) const
{
  auto velocity_x = nav_state.velocity_through_water.x;
  // Keep a small signed inflow so the angle calculation remains well-conditioned near rest.
  if (std::abs(velocity_x) < 0.05) {
    velocity_x = std::copysign(0.05, velocity_x == 0.0 ? 1.0 : velocity_x);
  }

  const double sideslip_angle = (velocity_x != 0.0)
    ? std::atan2(nav_state.velocity_through_water.y, velocity_x)
    : 0.0;

  const double course_angle = nav_state.attitude.yaw + sideslip_angle;
  const Eigen::Vector2d v_cm(
    std::cos(course_angle),
    std::sin(course_angle));
  const Eigen::Vector2d v_cm_scaled =
    v_cm * std::hypot(velocity_x, nav_state.velocity_through_water.y);

  const Eigen::Vector2d v_r(
    std::sin(nav_state.attitude.yaw),
    -std::cos(nav_state.attitude.yaw));
  const Eigen::Vector2d v_r_scaled =
    v_r * rudder_cm_distance_ * nav_state.angular_velocity.z;

  // Combine translational flow and the local velocity caused by yaw at the rudder.
  const Eigen::Vector2d v_s = v_cm_scaled + v_r_scaled;
  const double gamma = farol2_utils::wrapToPi(std::atan2(v_s(1), v_s(0)) - nav_state.attitude.yaw);

  RudderAllocationResult result;
  // Convert requested yaw torque to deflection, then enforce the mechanical stops.
  result.rudder_angle_rad = tau_r / (k_s_ * rudder_cm_distance_ * 1.75);
  result.rudder_angle_rad = (result.rudder_angle_rad > rudder_angle_max_)
    ? rudder_angle_max_
    : ((result.rudder_angle_rad < rudder_angle_min_) ? rudder_angle_min_ : result.rudder_angle_rad);

  result.flow_to_rudder_angle =
    result.rudder_angle_rad + ((v_s[0] != 0.0) ? std::atan2(v_s[1], v_s[0]) : 0.0) - nav_state.attitude.yaw;
  result.gamma = gamma;

  const double v_sq = v_s.dot(v_s);
  // Project rudder lift and drag onto the body surge axis for compensation.
  const double lift = k_l_ * result.flow_to_rudder_angle * v_sq;
  const double drag = (k_d0_ + k_d1_ * std::pow(result.flow_to_rudder_angle, 2)) * v_sq;
  result.rudder_x_body_drag = drag * std::cos(result.flow_to_rudder_angle)
    + lift * std::sin(-result.flow_to_rudder_angle);

  return result;
}

double RudderAllocator::solveDeltaFromTau(double tau_r, double gamma, double v_sq) const
{
  const double eps = 1e-12;

  const double a = k_d1_ * std::sin(gamma);
  const double b = k_l_ * std::cos(gamma);
  const double c = k_d0_ * std::sin(gamma) - tau_r / (rudder_cm_distance_ * v_sq);

  if (std::abs(a) < eps) {
    // Fall back to the linear equation when the quadratic term vanishes.
    if (std::abs(b) < eps) {
      return 0.0;
    }

    const double alpha = -c / b;
    return alpha - gamma;
  }

  double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0 && discriminant > -1e-12) {
    discriminant = 0.0;
  }

  if (discriminant < 0.0) {
    return 0.0;
  }

  const double sign_b = (b >= 0.0) ? 1.0 : -1.0;
  // This quadratic form avoids cancellation when b and sqrt(discriminant) are close.
  const double q = -0.5 * (b + sign_b * std::sqrt(discriminant));

  double alpha1;
  double alpha2;
  if (std::abs(q) < eps) {
    alpha1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
    alpha2 = (-b - std::sqrt(discriminant)) / (2.0 * a);
  } else {
    alpha1 = q / a;
    alpha2 = c / q;
  }

  const double d1 = alpha1 - gamma;
  const double d2 = alpha2 - gamma;
  return (std::abs(d1) <= std::abs(d2)) ? d1 : d2;
}
