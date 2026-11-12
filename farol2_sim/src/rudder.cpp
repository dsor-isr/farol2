#include "farol2_sim/actuators.hpp"
#include "farol2_utils/geometry.hpp"
#include "farol2_utils/validation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace farol2_sim {
Rudder::Rudder(const RudderParameters & p) : parameters_(p) {
  farol2_utils::requireOrderedBounds(p.minimum_angle, p.maximum_angle, "rudder angle limits");
  if (p.minimum_angle > 0.0 || p.maximum_angle < 0.0 || p.minimum_angle == p.maximum_angle) {
    throw std::invalid_argument("Rudder angle limits must include zero and have nonzero width");
  }
  farol2_utils::requireNonnegative(p.rate_limit, "rudder rate limit");
  farol2_utils::requireNonnegative(p.lift_coefficient, "rudder lift coefficient");
  farol2_utils::requireNonnegative(p.drag_constant, "rudder constant drag");
  farol2_utils::requireNonnegative(p.drag_quadratic, "rudder quadratic drag");
}

Vector6d Rudder::step(double command, double dt, const ActuatorGeometry & geometry,
  const State & s, const Environment &)
{
  farol2_utils::requireFinite(command, "rudder command");
  farol2_utils::requirePositive(dt, "rudder dt");
  command = std::clamp(command, parameters_.minimum_angle, parameters_.maximum_angle);
  const double delta = command - angle_;
  angle_ += parameters_.rate_limit == 0.0 ? delta :
    std::clamp(delta, -parameters_.rate_limit * dt, parameters_.rate_limit * dt);
  const Eigen::Vector3d flow = geometry.rotation_to_body.transpose() *
    farol2_utils::pointVelocity(s.velocity_through_water_body.head<3>(),
      s.velocity_through_water_body.tail<3>(), geometry.position_body);
  const double gamma = std::atan2(flow.y(), flow.x());
  const double alpha = std::remainder(angle_ + gamma, 2.0 * std::acos(-1.0));
  const double speed_squared = flow.head<2>().squaredNorm();
  const double lift = parameters_.lift_coefficient * alpha * speed_squared;
  const double drag = (parameters_.drag_constant + parameters_.drag_quadratic * alpha * alpha) * speed_squared;
  return farol2_utils::forceToWrench(Eigen::Vector3d(
    -drag * std::cos(gamma) + lift * std::sin(gamma),
    -drag * std::sin(gamma) - lift * std::cos(gamma), 0.0),
    geometry.position_body, geometry.rotation_to_body);
}
}  // namespace farol2_sim
