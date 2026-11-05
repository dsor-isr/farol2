#include "farol2_sim/actuators.hpp"
#include "farol2_utils/geometry.hpp"
#include "farol2_utils/validation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace farol2_sim {
Thruster::Thruster(const ThrusterParameters & p) : parameters_(p) {
  farol2_utils::requireOrderedBounds(p.minimum_rpm, p.maximum_rpm, "thruster RPM limits");
  if (p.minimum_rpm > 0.0 || p.maximum_rpm < 0.0 || p.minimum_rpm == p.maximum_rpm) {
    throw std::invalid_argument("Thruster RPM limits must include zero and have nonzero width");
  }
  farol2_utils::requireNonnegative(p.deadzone, "thruster deadzone");
  farol2_utils::requireNonnegative(p.pole, "thruster pole");
  farol2_utils::requireNonnegative(p.delay, "thruster delay");
  farol2_utils::requireFinite(p.positive_curve, "positive thrust curve");
  farol2_utils::requireFinite(p.negative_curve, "negative thrust curve");
  if (p.model == ThrusterParameters::Model::Propeller) {
    farol2_utils::requireNonnegative(p.thrust_coefficient, "propeller thrust coefficient");
    farol2_utils::requirePositive(p.diameter, "propeller diameter");
    farol2_utils::requirePositive(p.pitch, "propeller pitch");
    farol2_utils::requirePositive(p.thrust_scale, "propeller thrust scale");
  }
}

double Thruster::updateMotor(double command, double dt) {
  farol2_utils::requireFinite(command, "thruster command");
  farol2_utils::requirePositive(dt, "thruster dt");
  command = std::clamp(command, parameters_.minimum_rpm, parameters_.maximum_rpm);
  if (std::abs(command) < parameters_.deadzone) { command = 0.0; }
  commands_.emplace_back(time_ + parameters_.delay, command);
  const double end = time_ + dt;
  double cursor = time_;
  const auto advance = [this](double duration) {
    rpm_ = parameters_.pole == 0.0 ? delayed_command_ :
      delayed_command_ + (rpm_ - delayed_command_) * std::exp(-parameters_.pole * duration);
  };
  while (!commands_.empty() && commands_.front().first <= end) {
    const auto transition = commands_.front();
    if (transition.first > cursor) { advance(transition.first - cursor); }
    cursor = std::max(cursor, transition.first);
    delayed_command_ = transition.second;
    commands_.pop_front();
  }
  if (end > cursor) { advance(end - cursor); }
  time_ = end;
  return rpm_;
}

Vector6d Thruster::step(double command, double dt, const ActuatorGeometry & geometry,
  const State & s, const Environment & e)
{
  const double rpm = updateMotor(command, dt);
  double force = 0.0;
  if (rpm != 0.0) {
    if (parameters_.model == ThrusterParameters::Model::Polynomial) {
      const auto & curve = rpm > 0.0 ? parameters_.positive_curve : parameters_.negative_curve;
      force = curve[0] * rpm * rpm + curve[1] * std::abs(rpm) + curve[2];
    } else {
      const double rps = rpm / 60.0;
      // Propeller calibration uses vehicle through-water surge. Geometry still
      // determines the force axis and moment arm, but not the advance speed.
      const double surge = s.velocity_through_water_body[0];
      force = parameters_.thrust_scale * e.fluid_density * rps * std::abs(rps) *
        std::pow(parameters_.diameter, 4) * parameters_.thrust_coefficient *
        (1.0 - surge / (rps * parameters_.pitch));
    }
  }
  return farol2_utils::forceToWrench(Eigen::Vector3d(force, 0.0, 0.0),
    geometry.position_body, geometry.rotation_to_body);
}
}  // namespace farol2_sim
