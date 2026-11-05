#include "farol2_utils/geometry.hpp"
#include "farol2_utils/validation.hpp"
#include "farol2_sim/integrator.hpp"
#include "farol2_sim/kinematics.hpp"

namespace farol2_sim {
void validateState(const State & s) {
  farol2_utils::requireFinite(s.position_ned, "state position");
  farol2_utils::requireFinite(s.velocity_through_water_body, "state water-relative velocity");
  farol2_utils::requireRotationMatrix(s.rotation_body_to_ned, "state attitude");
}

Vector6d integrate(State & s, const Dynamics & dynamics, const Environment & environment,
  const Vector6d & wrench, double dt)
{
  farol2_utils::requirePositive(dt, "integration dt");
  validateState(s);
  const Vector6d acceleration = dynamics.acceleration(s, environment, wrench);
  State next = s;
  next.velocity_through_water_body = dynamics.constrainVelocity(s.velocity_through_water_body + dt * acceleration);
  next.position_ned += dt * positionDerivative(next, environment);
  next.rotation_body_to_ned = s.rotation_body_to_ned *
    farol2_utils::rotationIncrement(dt * next.velocity_through_water_body.tail<3>());
  validateState(next);
  s = next;
  return acceleration;
}
}  // namespace farol2_sim
