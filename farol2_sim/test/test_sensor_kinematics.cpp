#include <gtest/gtest.h>
#include "farol2_sim/sensor_kinematics.hpp"
#include "farol2_utils/geometry.hpp"

using namespace farol2_sim;

TEST(SensorKinematics, PositionAndAttitudeIncludeTheMount) {
  const double half_pi = std::acos(-1.0) / 2.0;
  State state;
  state.position_ned << 10.0, 20.0, 3.0;
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy({0.0, 0.0, half_pi});
  SensorMount mount;
  mount.position_body << 2.0, 0.0, 1.0;
  mount.rotation_to_body = farol2_utils::rotationFromRpy({0.0, 0.0, half_pi});
  EXPECT_TRUE(sensorPosition(state, mount).isApprox(Eigen::Vector3d(10.0, 22.0, 4.0)));
  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  expected(0, 0) = expected(1, 1) = -1.0;
  EXPECT_TRUE(sensorAttitude(state, mount).isApprox(expected));
}

TEST(SensorKinematics, DvlMountRotationLeverArmAndCurrentAreIndependent) {
  const double half_pi = std::acos(-1.0) / 2.0;
  State state;
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy({0.0, 0.0, half_pi});
  state.velocity_through_water_body << 1.0, 0.0, 0.0, 0.0, 0.0, 2.0;
  Environment environment;
  environment.current_ned << 0.0, 3.0, 0.0;
  SensorMount mount;
  mount.position_body << 2.0, 0.0, 0.0;
  mount.rotation_to_body = state.rotation_body_to_ned;
  EXPECT_TRUE(sensorVelocity(state, environment, mount, VelocityReference::Water)
    .isApprox(Eigen::Vector3d(4.0, -1.0, 0.0)));
  const auto ground = sensorVelocity(state, environment, mount, VelocityReference::Ground);
  EXPECT_TRUE(ground.isApprox(Eigen::Vector3d(4.0, -4.0, 0.0)));
  // Navigation's inverse mounting correction must recover body-origin velocity.
  const Eigen::Vector3d recovered = mount.rotation_to_body * ground -
    state.velocity_through_water_body.tail<3>().cross(mount.position_body);
  EXPECT_TRUE(recovered.isApprox(Eigen::Vector3d(4.0, 0.0, 0.0)));
}

TEST(SensorKinematics, DepthUsesRotatedVerticalOffset) {
  State state;
  state.position_ned.z() = 5.0;
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy({0.0, -std::acos(-1.0) / 2.0, 0.0});
  SensorMount mount;
  mount.position_body.x() = 2.0;
  EXPECT_NEAR(sensorPosition(state, mount).z(), 7.0, 1e-12);
}

TEST(SensorKinematics, StationaryAccelerometerMeasuresSpecificForceInSensorAxes) {
  State state;
  Environment environment;
  SensorMount mount;
  EXPECT_TRUE(sensorSpecificForce(state, Vector6d::Zero(), environment, mount)
    .isApprox(Eigen::Vector3d(0.0, 0.0, -environment.gravity)));
  mount.rotation_to_body = farol2_utils::rotationFromRpy({0.0, std::acos(-1.0) / 2.0, 0.0});
  EXPECT_TRUE(sensorSpecificForce(state, Vector6d::Zero(), environment, mount)
    .isApprox(Eigen::Vector3d(environment.gravity, 0.0, 0.0)));
}

TEST(SensorKinematics, ImuIncludesBodyDerivativeTangentialAndCentripetalAcceleration) {
  State state;
  state.velocity_through_water_body << 3.0, 0.0, 0.0, 0.0, 0.0, 2.0;
  Vector6d acceleration;
  acceleration << 0.0, 1.0, 2.0, 0.0, 0.0, 3.0;
  SensorMount mount;
  mount.position_body.x() = 1.0;
  mount.rotation_to_body = farol2_utils::rotationFromRpy({0.0, 0.0, std::acos(-1.0) / 2.0});
  Environment environment;
  const auto force = sensorSpecificForce(state, acceleration, environment, mount);
  EXPECT_TRUE(force.isApprox(Eigen::Vector3d(10.0, 4.0, 2.0 - environment.gravity)));
  environment.current_ned << 5.0, -2.0, 0.0;
  EXPECT_TRUE(sensorSpecificForce(state, acceleration, environment, mount).isApprox(force));
  mount.rotation_to_body = farol2_utils::rotationFromRpy({std::acos(-1.0) / 2.0, 0.0, 0.0});
  EXPECT_TRUE(sensorAngularVelocity(state, mount).isApprox(Eigen::Vector3d(0.0, 2.0, 0.0)));
}

TEST(SensorKinematics, OffsetAccelerationAgreesWithPositionSecondDerivative) {
  State state;
  state.velocity_through_water_body.tail<3>() << 0.3, -0.5, 0.8;
  SensorMount mount;
  mount.position_body << 1.0, 0.7, -0.3;
  const double dt = 1e-3;
  State before = state, after = state;
  const Eigen::Vector3d omega = state.velocity_through_water_body.tail<3>();
  before.rotation_body_to_ned = farol2_utils::rotationIncrement(-dt * omega);
  after.rotation_body_to_ned = farol2_utils::rotationIncrement(dt * omega);
  const Eigen::Vector3d numeric = (sensorPosition(after, mount) -
    2.0 * sensorPosition(state, mount) + sensorPosition(before, mount)) / (dt * dt);
  Environment environment;
  const Eigen::Vector3d inertial = sensorSpecificForce(state, Vector6d::Zero(), environment, mount) +
    Eigen::Vector3d(0.0, 0.0, environment.gravity);
  EXPECT_TRUE(numeric.isApprox(inertial, 1e-6));
}
