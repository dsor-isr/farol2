#include "farol2_utils/geometry.hpp"
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <limits>
#include "farol2_sim/actuators.hpp"
#include "farol2_sim/integrator.hpp"
#include "farol2_sim/kinematics.hpp"

using namespace farol2_sim;

TEST(Dynamics, WrenchAndMassGiveAcceleration) {
  DynamicsParameters p;
  p.mass = 10.0;
  p.inertia.diagonal() << 2.0, 3.0, 4.0;
  p.added_mass.diagonal() << 2.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  p.displaced_volume = p.mass / 1025.0;
  Dynamics dynamics(p);
  Vector6d wrench = Vector6d::Zero();
  wrench[0] = 24.0; wrench[5] = 10.0;
  Vector6d expected = Vector6d::Zero(); expected[0] = 2.0; expected[5] = 2.0;
  EXPECT_TRUE(dynamics.acceleration(State{}, Environment{}, wrench).isApprox(expected));
}

TEST(Dynamics, CoriolisIsSkewAndDoesNoWork) {
  Matrix6d m = Matrix6d::Identity(); m(0, 5) = m(5, 0) = 0.2;
  Vector6d velocity; velocity << 1.0, 2.0, 3.0, 0.4, 0.5, 0.6;
  const Matrix6d c = Dynamics::coriolis(m, velocity);
  EXPECT_TRUE(c.isApprox(-c.transpose()));
  EXPECT_NEAR(velocity.dot(c * velocity), 0.0, 1e-12);
}

TEST(Dynamics, ReducedSolveIncludesConstraintReactions) {
  DynamicsParameters p;
  p.added_mass(0, 0) = p.added_mass(1, 1) = 1.0;
  p.added_mass(0, 1) = p.added_mass(1, 0) = 0.5;
  p.active_dofs = {0};
  Dynamics dynamics(p);
  Vector6d wrench = Vector6d::Zero(); wrench[0] = 2.0; wrench[1] = 100.0;
  const auto a = dynamics.acceleration(State{}, Environment{}, wrench);
  EXPECT_DOUBLE_EQ(a[0], 1.0);
  EXPECT_DOUBLE_EQ(a.tail<5>().norm(), 0.0);
  State state;
  integrate(state, dynamics, Environment{}, wrench, 0.1);
  EXPECT_DOUBLE_EQ(state.position_ned.y(), 0.0);
  EXPECT_DOUBLE_EQ(state.position_ned.z(), 0.0);
}

TEST(Dynamics, HydrostaticsUseBothCentersAndAttitude) {
  DynamicsParameters p;
  p.mass = 10.0; p.displaced_volume = 10.0 / 1025.0;
  p.center_of_gravity = Eigen::Vector3d(0.1, 0.0, 0.0);
  p.center_of_buoyancy = Eigen::Vector3d(-0.1, 0.0, 0.0);
  Dynamics dynamics(p);
  const Vector6d g = dynamics.restoring(State{}, Environment{});
  EXPECT_NEAR(g.head<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(g[4], 19.6, 1e-12);
  State state; state.rotation_body_to_ned = farol2_utils::rotationFromRpy(Eigen::Vector3d(0.3, 0.2, 0.1));
  EXPECT_TRUE(dynamics.restoring(state, Environment{}).allFinite());
}

TEST(Dynamics, WaterFollowingVehicleHasNoHydrodynamicDrag) {
  DynamicsParameters p;
  p.displaced_volume = p.mass / 1025.0;
  p.linear_damping = 10.0 * Matrix6d::Identity();
  p.added_mass = Matrix6d::Identity();
  Dynamics dynamics(p);
  State state; Environment environment;
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy(Eigen::Vector3d(0.0, 0.0, 0.8));
  environment.current_ned = Eigen::Vector3d(1.0, 0.2, 0.0);
  EXPECT_NEAR(state.velocity_through_water_body.norm(), 0.0, 1e-12);
  for (int i = 0; i < 1000; ++i) { integrate(state, dynamics, environment, Vector6d::Zero(), 0.01); }
  EXPECT_TRUE(state.position_ned.isApprox(10.0 * environment.current_ned, 1e-10));
  EXPECT_NEAR(state.velocity_through_water_body.norm(), 0.0, 1e-12);
}

TEST(Dynamics, SurfaceBuoyancyUsesWorldWaterLevel) {
  DynamicsParameters p;
  p.displaced_volume = p.mass / 1025.0;
  p.buoyancy_sphere_radius = 1.0;
  Dynamics dynamics(p);
  State s; Environment e;
  EXPECT_NEAR(dynamics.restoring(s, e).norm(), 0.0, 1e-12);
  e.water_surface_z = 0.5;  // Body at z=0 is now above the water surface.
  const Vector6d above_water = dynamics.restoring(s, e);
  EXPECT_NEAR(above_water[2], -p.mass * e.gravity * (1.0 - 0.125), 1e-12);
  s.position_ned.z() += 10.0;
  e.water_surface_z += 10.0;
  EXPECT_TRUE(dynamics.restoring(s, e).isApprox(above_water));
}

TEST(Dynamics, ChangingCurrentOnlyChangesKinematics) {
  DynamicsParameters p;
  p.displaced_volume = p.mass / 1025.0;
  p.added_mass = 2.0 * Matrix6d::Identity();
  p.linear_damping = Matrix6d::Identity();
  Dynamics dynamics(p);
  State state; Environment environment;
  state.velocity_through_water_body << 1.0, 0.3, -0.2, 0.1, 0.2, 0.5;
  const Vector6d acceleration = dynamics.acceleration(state, environment, Vector6d::Zero());
  environment.current_ned << 3.0, -1.0, 0.5;
  EXPECT_TRUE(dynamics.acceleration(state, environment, Vector6d::Zero()).isApprox(acceleration));
  State first = state, second = state;
  integrate(first, dynamics, Environment{}, Vector6d::Zero(), 0.01);
  integrate(second, dynamics, environment, Vector6d::Zero(), 0.01);
  EXPECT_TRUE(first.velocity_through_water_body.isApprox(second.velocity_through_water_body));
  EXPECT_TRUE(first.rotation_body_to_ned.isApprox(second.rotation_body_to_ned));
  EXPECT_TRUE((second.position_ned - first.position_ned).isApprox(0.01 * environment.current_ned));
}

TEST(Kinematics, DvlReferenceAndFrameAreIndependent) {
  State state; Environment environment;
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy(Eigen::Vector3d(0.0, 0.0, std::acos(-1.0) / 2.0));
  state.velocity_through_water_body.head<3>() << 2.0, 0.5, 0.1;
  environment.current_ned << 1.0, -0.25, 0.2;
  const Eigen::Vector3d water_body(2.0, 0.5, 0.1);
  const Eigen::Vector3d water_ned(-0.5, 2.0, 0.1);
  const Eigen::Vector3d ground_body(1.75, -0.5, 0.3);
  const Eigen::Vector3d ground_ned(0.5, 1.75, 0.3);
  EXPECT_TRUE(linearVelocity(state, environment, VelocityReference::Water, VelocityFrame::Body).isApprox(water_body));
  EXPECT_TRUE(linearVelocity(state, environment, VelocityReference::Water, VelocityFrame::Ned).isApprox(water_ned));
  EXPECT_TRUE(linearVelocity(state, environment, VelocityReference::Ground, VelocityFrame::Body).isApprox(ground_body));
  EXPECT_TRUE(linearVelocity(state, environment, VelocityReference::Ground, VelocityFrame::Ned).isApprox(ground_ned));
  EXPECT_TRUE(positionDerivative(state, environment).isApprox(ground_ned));
  // A drifting vehicle has zero water-track speed but nonzero bottom-track velocity.
  state.velocity_through_water_body.setZero();
  EXPECT_DOUBLE_EQ(linearVelocity(state, environment, VelocityReference::Water, VelocityFrame::Body).norm(), 0.0);
  EXPECT_TRUE(positionDerivative(state, environment).isApprox(environment.current_ned));
}

TEST(Kinematics, CurrentIsAddedOnceForConstrainedVehicle) {
  DynamicsParameters p; p.active_dofs = {0, 1, 5};
  p.displaced_volume = p.mass / 1025.0;
  Dynamics dynamics(p);
  State state; Environment environment;
  environment.current_ned << 0.2, -0.1, 0.0;
  for (int i = 0; i < 1000; ++i) {
    integrate(state, dynamics, environment, Vector6d::Zero(), 0.01);
  }
  EXPECT_DOUBLE_EQ(state.velocity_through_water_body.norm(), 0.0);
  EXPECT_TRUE(state.position_ned.isApprox(10.0 * environment.current_ned, 1e-10));
}

TEST(Kinematics, DvlModesCoincideWithoutCurrent) {
  State state;
  state.rotation_body_to_ned = farol2_utils::rotationFromRpy(Eigen::Vector3d(0.2, -0.4, 0.7));
  state.velocity_through_water_body.head<3>() << 2.0, -0.5, 0.1;
  for (const auto frame : {VelocityFrame::Body, VelocityFrame::Ned}) {
    EXPECT_TRUE(linearVelocity(state, Environment{}, VelocityReference::Water, frame).isApprox(
      linearVelocity(state, Environment{}, VelocityReference::Ground, frame)));
  }
}

TEST(Dynamics, DampingDissipatesRelativeMotion) {
  DynamicsParameters p;
  p.displaced_volume = p.mass / 1025.0;
  p.linear_damping = Matrix6d::Identity();
  p.quadratic_damping = 2.0 * Matrix6d::Identity();
  State s; s.velocity_through_water_body[0] = -2.0;
  const auto a = Dynamics(p).acceleration(s, Environment{}, Vector6d::Zero());
  EXPECT_NEAR(a[0], 10.0, 1e-12);
  EXPECT_LT(s.velocity_through_water_body.dot(a), 0.0);
}

TEST(Kinematics, RotationStaysOnSO3AcrossPitchSingularity) {
  DynamicsParameters p;
  p.displaced_volume = p.mass / 1025.0;
  Dynamics dynamics(p);
  State state;
  state.velocity_through_water_body.tail<3>() << 0.3, 0.8, 0.2;
  const Eigen::Vector3d omega = state.velocity_through_water_body.tail<3>();
  for (int i = 0; i < 20000; ++i) { integrate(state, dynamics, Environment{}, Vector6d::Zero(), 0.001); }
  EXPECT_TRUE((state.rotation_body_to_ned.transpose() * state.rotation_body_to_ned)
    .isApprox(Eigen::Matrix3d::Identity(), 1e-10));
  EXPECT_NEAR(state.rotation_body_to_ned.determinant(), 1.0, 1e-10);
  EXPECT_TRUE(state.rotation_body_to_ned.isApprox(farol2_utils::rotationIncrement(20.0 * omega), 1e-10));
}

TEST(Validation, RejectsInvalidModelsAndStates) {
  DynamicsParameters p; p.active_dofs = {0, 0};
  EXPECT_THROW(Dynamics{p}, std::invalid_argument);
  p.active_dofs = {6}; EXPECT_THROW(Dynamics{p}, std::invalid_argument);
  p.active_dofs = {0}; p.mass = 0.0; EXPECT_THROW(Dynamics{p}, std::invalid_argument);
  p.mass = 1.0; p.inertia(0, 0) = -1.0; EXPECT_THROW(Dynamics{p}, std::invalid_argument);
  p.inertia.setIdentity(); p.linear_damping(0, 0) = -1.0;
  EXPECT_THROW(Dynamics{p}, std::invalid_argument);
  Dynamics dynamics(DynamicsParameters{});
  State state;
  EXPECT_THROW(integrate(state, dynamics, Environment{}, Vector6d::Zero(), 0.0), std::invalid_argument);
  state.rotation_body_to_ned(0, 0) = 2.0;
  EXPECT_THROW(integrate(state, dynamics, Environment{}, Vector6d::Zero(), 0.1), std::invalid_argument);
}

TEST(Actuators, ThrusterUsesAxisAndMomentArm) {
  ThrusterParameters p;
  p.positive_curve << 0.01, 0.0, 0.0;
  p.negative_curve << -0.01, 0.0, 0.0;
  Thruster thruster(p);
  ActuatorGeometry g; g.position_body << -1.0, 2.0, 0.0;
  auto wrench = thruster.step(10.0, 0.1, g, State{}, Environment{});
  EXPECT_DOUBLE_EQ(wrench[0], 1.0); EXPECT_DOUBLE_EQ(wrench[5], -2.0);
  g.rotation_to_body = farol2_utils::rotationFromRpy(Eigen::Vector3d(0.0, -std::acos(-1.0) / 2.0, 0.0));
  wrench = thruster.step(-10.0, 0.1, g, State{}, Environment{});
  EXPECT_NEAR(wrench[2], -1.0, 1e-12);
  EXPECT_NEAR(wrench[3], -2.0, 1e-12);
}

TEST(Actuators, DelayAndLagUseElapsedTimeIncludingFractionalDelay) {
  ThrusterParameters p; p.pole = 2.0; p.delay = 0.15;
  Thruster thruster(p); ActuatorGeometry g;
  thruster.step(100.0, 0.1, g, State{}, Environment{});
  EXPECT_DOUBLE_EQ(thruster.rpm(), 0.0);
  thruster.step(100.0, 0.1, g, State{}, Environment{});
  EXPECT_NEAR(thruster.rpm(), 100.0 * (1.0 - std::exp(-0.1)), 1e-12);
  thruster.step(100.0, 0.2, g, State{}, Environment{});
  EXPECT_NEAR(thruster.rpm(), 100.0 * (1.0 - std::exp(-0.5)), 1e-12);
}

TEST(Actuators, ZeroCommandAndDeadzoneProduceNoConstantCurveThrust) {
  ThrusterParameters p; p.deadzone = 10.0; p.positive_curve << 1.0, 0.0, 20.0;
  Thruster thruster(p);
  EXPECT_DOUBLE_EQ(thruster.step(5.0, 0.1, ActuatorGeometry{}, State{}, Environment{}).norm(), 0.0);
  EXPECT_THROW(thruster.step(std::numeric_limits<double>::quiet_NaN(), 0.1,
    ActuatorGeometry{}, State{}, Environment{}), std::invalid_argument);
}

TEST(Actuators, PropellerRespondsToWaterRelativeAdvanceSpeed) {
  ThrusterParameters p; p.model = ThrusterParameters::Model::Propeller;
  p.thrust_coefficient = 0.1; p.pitch = 0.5; p.diameter = 0.4;
  Thruster thruster(p); State s; Environment e;
  s.velocity_through_water_body[0] = 0.5; e.current_ned.x() = 2.0;
  const auto wrench = thruster.step(120.0, 0.1, ActuatorGeometry{}, s, e);
  EXPECT_NEAR(wrench[0], 1025.0 * 4.0 * std::pow(0.4, 4) * 0.1 * 0.5, 1e-12);
}

TEST(Actuators, EqualRpmUsesCommonSurgeDuringYawWithoutLosingMomentArms) {
  ThrusterParameters p; p.model = ThrusterParameters::Model::Propeller;
  p.thrust_coefficient = 0.1; p.pitch = 0.5; p.diameter = 0.4;
  Thruster port(p), starboard(p);
  State s;
  s.velocity_through_water_body << 0.5, 0.3, -0.2, 0.1, 0.2, 0.4;
  ActuatorGeometry left, right;
  left.position_body << -4.0, -1.5, 0.0;
  right.position_body << -4.0, 1.5, 0.0;
  const Vector6d a = port.step(120.0, 0.1, left, s, Environment{});
  const Vector6d b = starboard.step(120.0, 0.1, right, s, Environment{});
  EXPECT_GT(a[0], 0.0);
  EXPECT_DOUBLE_EQ(a[0], b[0]);
  EXPECT_NEAR(a[5], 1.5 * a[0], 1e-12);
  EXPECT_NEAR(b[5], -1.5 * b[0], 1e-12);
  EXPECT_NEAR((a + b)[5], 0.0, 1e-12);
  // Rotating the actuator changes the applied force axis, not its thrust magnitude.
  right.rotation_to_body = farol2_utils::rotationFromRpy(Eigen::Vector3d(0.0, std::acos(-1.0) / 2.0, 0.0));
  const Vector6d rotated = starboard.step(120.0, 0.1, right, s, Environment{});
  EXPECT_NEAR(rotated.head<3>().norm(), a[0], 1e-12);
  EXPECT_NEAR(rotated[2], -a[0], 1e-12);
}

TEST(Actuators, RudderHasSwayForceYawMomentAndRateLimit) {
  RudderParameters p; p.lift_coefficient = 2.0; p.rate_limit = 0.1;
  Rudder rudder(p); ActuatorGeometry g; g.position_body.x() = -4.0;
  State s; s.velocity_through_water_body[0] = 2.0;
  const auto wrench = rudder.step(0.5, 0.1, g, s, Environment{});
  EXPECT_NEAR(rudder.angle(), 0.01, 1e-12);
  EXPECT_NEAR(wrench[1], -0.08, 1e-12);
  EXPECT_NEAR(wrench[5], 0.32, 1e-12);
  Environment current; current.current_ned << 3.0, -1.0, 0.5;
  Rudder same_rudder(p);
  EXPECT_TRUE(same_rudder.step(0.5, 0.1, g, s, current).isApprox(wrench));
  s.velocity_through_water_body.setZero();
  EXPECT_DOUBLE_EQ(rudder.step(0.5, 0.1, g, s, Environment{}).norm(), 0.0);
}
