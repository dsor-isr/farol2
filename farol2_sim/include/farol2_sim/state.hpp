#pragma once

#include <Eigen/Core>

namespace farol2_sim {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct State {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation_body_to_ned{Eigen::Matrix3d::Identity()};
  // Water-relative linear velocity (surge/sway/heave) and angular velocity,
  // both in body axes. Ground velocity is derived by the kinematics.
  Vector6d velocity_through_water_body{Vector6d::Zero()};
};

struct Environment {
  double fluid_density{1025.0};
  double gravity{9.8};
  double water_surface_z{0.0};  // Local NED surface coordinate, m.
  Eigen::Vector3d current_ned{Eigen::Vector3d::Zero()};
};

}  // namespace farol2_sim
