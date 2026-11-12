#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace farol2_utils {

/// Cross-product matrix: skew(v) * w == v.cross(w).
inline Eigen::Matrix3d skew(const Eigen::Vector3d & v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

/// SO(3) exponential of a rotation vector (axis times angle in radians).
inline Eigen::Matrix3d rotationIncrement(const Eigen::Vector3d & vector) {
  const double angle = vector.norm();
  if (angle < 1e-8) {
    const Eigen::Matrix3d k = skew(vector);
    return Eigen::Matrix3d::Identity() + (1.0 - angle * angle / 6.0) * k +
      (0.5 - angle * angle / 24.0) * k * k;
  }
  return Eigen::AngleAxisd(angle, vector / angle).toRotationMatrix();
}

/// RPY radians to rotation matrix, using Rz(yaw) * Ry(pitch) * Rx(roll).
inline Eigen::Matrix3d rotationFromRpy(const Eigen::Vector3d & rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// Rotation matrix to principal RPY radians, for presentation rather than integration.
inline Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & r) {
  return Eigen::Vector3d(std::atan2(r(2, 1), r(2, 2)),
    std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0))), std::atan2(r(1, 0), r(0, 0)));
}

/// Velocity at an offset point; all inputs and output are expressed in the same frame.
inline Eigen::Vector3d pointVelocity(const Eigen::Vector3d & linear_velocity,
  const Eigen::Vector3d & angular_velocity, const Eigen::Vector3d & position)
{
  return linear_velocity + angular_velocity.cross(position);
}

/// Map a local force to [body force; body moment] about the body origin.
/// position_body is the force application point; rotation_local_to_body maps local axes to body axes.
inline Eigen::Matrix<double, 6, 1> forceToWrench(const Eigen::Vector3d & local_force,
  const Eigen::Vector3d & position_body, const Eigen::Matrix3d & rotation_local_to_body)
{
  Eigen::Matrix<double, 6, 1> wrench;
  wrench.head<3>() = rotation_local_to_body * local_force;
  wrench.tail<3>() = position_body.cross(wrench.head<3>());
  return wrench;
}

}  // namespace farol2_utils
