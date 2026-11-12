#pragma once

#include <vector>
#include "farol2_sim/state.hpp"

namespace farol2_sim {
struct DynamicsParameters {
  double mass{1.0};
  // Inertia about CG, expressed in body axes.
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d center_of_gravity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_of_buoyancy{Eigen::Vector3d::Zero()};
  Matrix6d added_mass{Matrix6d::Zero()};
  Matrix6d linear_damping{Matrix6d::Zero()};
  // Force convention: D_l * nu_r + D_q * (abs(nu_r) .* nu_r).
  Matrix6d quadratic_damping{Matrix6d::Zero()};
  double displaced_volume{0.0};
  // Optional legacy surface approximation; zero selects constant volume.
  double buoyancy_sphere_radius{0.0};
  // Body velocity components allowed to evolve: u,v,w,p,q,r -> 0..5.
  std::vector<int> active_dofs{0, 1, 2, 3, 4, 5};
};

class Dynamics {
public:
  explicit Dynamics(const DynamicsParameters & parameters);
  Vector6d acceleration(const State &, const Environment &, const Vector6d & wrench) const;
  Vector6d restoring(const State &, const Environment &) const;
  Vector6d constrainVelocity(const Vector6d &) const;
  const Matrix6d & massMatrix() const { return mass_; }
  static Matrix6d coriolis(const Matrix6d & mass, const Vector6d & velocity);

private:
  DynamicsParameters parameters_;
  Matrix6d rigid_body_mass_{Matrix6d::Zero()};
  Matrix6d mass_;
  Eigen::MatrixXd selection_;
  Eigen::MatrixXd reduced_mass_;
};
}  // namespace farol2_sim
