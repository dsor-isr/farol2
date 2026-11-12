#include "farol2_sim/sensor_kinematics.hpp"
#include "farol2_utils/geometry.hpp"

namespace farol2_sim {

Eigen::Vector3d sensorPosition(const State & state, const SensorMount & mount) {
  return state.position_ned + state.rotation_body_to_ned * mount.position_body;
}

Eigen::Matrix3d sensorAttitude(const State & state, const SensorMount & mount) {
  return state.rotation_body_to_ned * mount.rotation_to_body;
}

Eigen::Vector3d sensorVelocity(const State & state, const Environment & environment,
  const SensorMount & mount, VelocityReference reference) {
  const Eigen::Vector3d velocity_body = linearVelocity(state, environment, reference, VelocityFrame::Body);
  return mount.rotation_to_body.transpose() * farol2_utils::pointVelocity(
    velocity_body, state.velocity_through_water_body.tail<3>(), mount.position_body);
}

Eigen::Vector3d sensorAngularVelocity(const State & state, const SensorMount & mount) {
  return mount.rotation_to_body.transpose() * state.velocity_through_water_body.tail<3>();
}

Eigen::Vector3d sensorSpecificForce(const State & state, const Vector6d & acceleration,
  const Environment & environment, const SensorMount & mount) {
  const Eigen::Vector3d omega = state.velocity_through_water_body.tail<3>();
  // The model integrates body-coordinate water-relative velocity. For a uniform
  // constant NED current, its inertial derivative contributes no acceleration.
  const Eigen::Vector3d acceleration_body = acceleration.head<3>() +
    omega.cross(state.velocity_through_water_body.head<3>()) +
    acceleration.tail<3>().cross(mount.position_body) +
    omega.cross(omega.cross(mount.position_body));
  const Eigen::Vector3d gravity_body = state.rotation_body_to_ned.transpose() *
    Eigen::Vector3d(0.0, 0.0, environment.gravity);
  return mount.rotation_to_body.transpose() * (acceleration_body - gravity_body);
}

}  // namespace farol2_sim
