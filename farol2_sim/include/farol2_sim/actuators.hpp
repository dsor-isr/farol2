#pragma once

#include <deque>
#include <utility>
#include "farol2_sim/state.hpp"

namespace farol2_sim {
struct ActuatorGeometry {
  Eigen::Vector3d position_body{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation_to_body{Eigen::Matrix3d::Identity()};
};


struct ThrusterParameters {
  enum class Model { Polynomial, Propeller };
  Model model{Model::Polynomial};
  double minimum_rpm{-5000.0}, maximum_rpm{5000.0};
  double deadzone{0.0}, pole{0.0}, delay{0.0};
  Eigen::Vector3d positive_curve{Eigen::Vector3d::Zero()};
  Eigen::Vector3d negative_curve{Eigen::Vector3d::Zero()};
  double thrust_coefficient{0.0}, diameter{0.0}, pitch{0.0}, thrust_scale{1.0};
};

class Thruster {
public:
  explicit Thruster(const ThrusterParameters & parameters);
  Vector6d step(double command_rpm, double dt, const ActuatorGeometry &,
    const State &, const Environment &);
  double rpm() const { return rpm_; }

private:
  double updateMotor(double command, double dt);
  ThrusterParameters parameters_;
  double time_{0.0}, delayed_command_{0.0}, rpm_{0.0};
  std::deque<std::pair<double, double>> commands_;
};

struct RudderParameters {
  double minimum_angle{-0.6}, maximum_angle{0.6};
  double rate_limit{0.2};  // rad/s, zero means ideal angle tracking
  double lift_coefficient{0.0}, drag_constant{0.0}, drag_quadratic{0.0};
};

class Rudder {
public:
  explicit Rudder(const RudderParameters & parameters);
  Vector6d step(double angle_command, double dt, const ActuatorGeometry &,
    const State &, const Environment &);
  double angle() const { return angle_; }

private:
  RudderParameters parameters_;
  double angle_{0.0};
};
}  // namespace farol2_sim
