#include "farol2_utils/geometry.hpp"
#include "farol2_sim/kinematics.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace farol2_sim {
Eigen::Vector3d linearVelocity(const State & s, const Environment & e,
  VelocityReference reference, VelocityFrame frame)
{
  if (frame == VelocityFrame::Ned) {
    Eigen::Vector3d velocity = s.rotation_body_to_ned * s.velocity_through_water_body.head<3>();
    if (reference == VelocityReference::Ground) { velocity += e.current_ned; }
    return velocity;
  }
  Eigen::Vector3d velocity = s.velocity_through_water_body.head<3>();
  if (reference == VelocityReference::Ground) { velocity += s.rotation_body_to_ned.transpose() * e.current_ned; }
  return velocity;
}

Eigen::Vector3d positionDerivative(const State & s, const Environment & e) {
  return linearVelocity(s, e, VelocityReference::Ground, VelocityFrame::Ned);
}

Eigen::Matrix3d rotationDerivative(const State & s) {
  return s.rotation_body_to_ned * farol2_utils::skew(s.velocity_through_water_body.tail<3>());
}

}  // namespace farol2_sim
