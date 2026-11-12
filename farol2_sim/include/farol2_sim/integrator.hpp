#pragma once

#include "farol2_sim/dynamics.hpp"

namespace farol2_sim {
// Semi-implicit velocity/position step and SO(3) exponential attitude step.
// Input velocity must satisfy the dynamics' body-component constraints.
Vector6d integrate(State &, const Dynamics &, const Environment &, const Vector6d & wrench, double dt);
void validateState(const State &);
}  // namespace farol2_sim
