#pragma once

#include "farol2_sim/kinematics.hpp"

namespace farol2_sim {

// Fixed mounting transform: sensor origin in body coordinates and sensor axes
// mapped into body axes. Filled from base_frame <- sensor_frame TF.
struct SensorMount {
  Eigen::Vector3d position_body{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation_to_body{Eigen::Matrix3d::Identity()};
};

Eigen::Vector3d sensorPosition(const State &, const SensorMount &);
Eigen::Matrix3d sensorAttitude(const State &, const SensorMount &);
Eigen::Vector3d sensorVelocity(const State &, const Environment &, const SensorMount &,
  VelocityReference);  // Sensor-origin velocity, expressed in sensor axes.
Eigen::Vector3d sensorAngularVelocity(const State &, const SensorMount &);
Eigen::Vector3d sensorSpecificForce(const State &, const Vector6d & acceleration,
  const Environment &, const SensorMount &);

}  // namespace farol2_sim
