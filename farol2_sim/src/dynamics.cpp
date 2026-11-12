#include "farol2_utils/geometry.hpp"
#include "farol2_utils/validation.hpp"
#include "farol2_sim/dynamics.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace farol2_sim {
Dynamics::Dynamics(const DynamicsParameters & p) : parameters_(p) {
  farol2_utils::requirePositive(p.mass, "mass");
  farol2_utils::requireFinite(p.center_of_gravity, "center of gravity");
  farol2_utils::requireFinite(p.center_of_buoyancy, "center of buoyancy");
  farol2_utils::requireNonnegative(p.displaced_volume, "displaced volume");
  farol2_utils::requireNonnegative(p.buoyancy_sphere_radius, "buoyancy sphere radius");
  farol2_utils::requirePositiveDefinite(p.inertia, "inertia");
  farol2_utils::requirePositiveSemidefinite(p.added_mass, "added mass");
  farol2_utils::requirePositiveSemidefinite(p.linear_damping, "linear damping");
  // Nonlinear cross damping is not necessarily dissipative. Use a diagonal
  // nonnegative quadratic damping matrix until a richer model is specified.
  farol2_utils::requirePositiveSemidefinite(p.quadratic_damping, "quadratic damping");
  if (!p.quadratic_damping.isDiagonal(1e-12)) {
    throw std::invalid_argument("quadratic damping must be diagonal");
  }
  const Eigen::Matrix3d sg = farol2_utils::skew(p.center_of_gravity);
  rigid_body_mass_.topLeftCorner<3, 3>() = p.mass * Eigen::Matrix3d::Identity();
  rigid_body_mass_.topRightCorner<3, 3>() = -p.mass * sg;
  rigid_body_mass_.bottomLeftCorner<3, 3>() = p.mass * sg;
  rigid_body_mass_.bottomRightCorner<3, 3>() = p.inertia - p.mass * sg * sg;
  mass_ = rigid_body_mass_ + p.added_mass;
  farol2_utils::requirePositiveDefinite(mass_, "total mass");
  if (p.active_dofs.empty() || p.active_dofs.size() > 6) {
    throw std::invalid_argument("active_dofs must contain 1 to 6 unique indices");
  }
  farol2_utils::requireUnique(p.active_dofs, "active_dofs");
  selection_ = Eigen::MatrixXd::Zero(6, p.active_dofs.size());
  for (size_t i = 0; i < p.active_dofs.size(); ++i) {
    const int index = p.active_dofs[i];
    if (index < 0 || index > 5) {
      throw std::invalid_argument("active_dofs must contain unique indices in 0..5");
    }
    selection_(index, i) = 1.0;
  }
  reduced_mass_ = selection_.transpose() * mass_ * selection_;
}

Matrix6d Dynamics::coriolis(const Matrix6d & mass, const Vector6d & velocity) {
  const Vector6d momentum = mass * velocity;
  Matrix6d c = Matrix6d::Zero();
  c.topRightCorner<3, 3>() = -farol2_utils::skew(momentum.head<3>());
  c.bottomLeftCorner<3, 3>() = -farol2_utils::skew(momentum.head<3>());
  c.bottomRightCorner<3, 3>() = -farol2_utils::skew(momentum.tail<3>());
  return c;
}

Vector6d Dynamics::restoring(const State & s, const Environment & e) const {
  double volume = parameters_.displaced_volume;
  if (parameters_.buoyancy_sphere_radius > 0.0) {
    const double radius = parameters_.buoyancy_sphere_radius;
    const double depth = s.position_ned.z() - e.water_surface_z;
    const double fraction = std::clamp((radius + depth) / radius, 0.0, 1.0);
    volume *= fraction * fraction * fraction;
  }
  const Eigen::Vector3d weight = s.rotation_body_to_ned.transpose() *
    Eigen::Vector3d(0.0, 0.0, parameters_.mass * e.gravity);
  const Eigen::Vector3d buoyancy = s.rotation_body_to_ned.transpose() *
    Eigen::Vector3d(0.0, 0.0, -e.fluid_density * volume * e.gravity);
  Vector6d g;
  g.head<3>() = -weight - buoyancy;
  g.tail<3>() = -parameters_.center_of_gravity.cross(weight) -
    parameters_.center_of_buoyancy.cross(buoyancy);
  return g;
}

Vector6d Dynamics::constrainVelocity(const Vector6d & velocity) const {
  return selection_ * (selection_.transpose() * velocity);
}

Vector6d Dynamics::acceleration(const State & s, const Environment & e, const Vector6d & wrench) const {
  farol2_utils::requireFinite(s.velocity_through_water_body, "water-relative velocity");
  farol2_utils::requireFinite(wrench, "body wrench");
  farol2_utils::requirePositive(e.fluid_density, "fluid density");
  farol2_utils::requirePositive(e.gravity, "gravity");
  farol2_utils::requireFinite(e.water_surface_z, "water surface coordinate");
  if (!s.velocity_through_water_body.isApprox(constrainVelocity(s.velocity_through_water_body), 1e-10)) {
    throw std::invalid_argument("Velocity violates active_dofs constraints");
  }
  // Use velocity through water for the dynamics model 
  const Vector6d & velocity = s.velocity_through_water_body;
  // Compute total drag
  const Vector6d damping = parameters_.linear_damping * velocity + parameters_.quadratic_damping * velocity.cwiseAbs().cwiseProduct(velocity);
  // Compute right hand side of the fossen dynamics equation
  const Vector6d rhs = wrench - coriolis(mass_, velocity) * velocity - damping - restoring(s, e);
  // Solve the reduced equations, rather than zeroing a full-system solution.
  const Eigen::VectorXd reduced_rhs = selection_.transpose() * rhs;
  return selection_ * reduced_mass_.ldlt().solve(reduced_rhs);
}
}  // namespace farol2_sim
