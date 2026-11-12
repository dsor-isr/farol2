#pragma once

#include "farol2_sim/state.hpp"

namespace farol2_sim {
enum class VelocityReference { Water, Ground };
enum class VelocityFrame { Body, Ned };
// Body-origin velocity for truth and sensor output. Current is added once,
// only for ground-relative output; frame conversion is independent of mode.
Eigen::Vector3d linearVelocity(const State &, const Environment &, VelocityReference, VelocityFrame);
Eigen::Vector3d positionDerivative(const State & state, const Environment & environment);
Eigen::Matrix3d rotationDerivative(const State & state);
}  // namespace farol2_sim
