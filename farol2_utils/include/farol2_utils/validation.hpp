#pragma once

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <set>
#include <string>
#include <type_traits>

namespace farol2_utils {

template<typename Scalar, std::enable_if_t<std::is_arithmetic_v<Scalar>, int> = 0>
void requireFinite(Scalar value, const std::string & name) {
  if (!std::isfinite(value)) { throw std::invalid_argument(name + " must be finite"); }
}

template<typename Derived>
void requireFinite(const Eigen::MatrixBase<Derived> & values, const std::string & name) {
  if (!values.allFinite()) { throw std::invalid_argument(name + " must contain finite values"); }
}

template<typename Range>
void requireFiniteValues(const Range & values, const std::string & name) {
  for (const auto & value : values) { requireFinite(value, name); }
}

template<typename Container>
void requireSize(const Container & values, std::size_t expected, const std::string & name) {
  if (static_cast<std::size_t>(values.size()) != expected) {
    throw std::invalid_argument(name + " must have " + std::to_string(expected) + " values");
  }
}

template<typename Container>
void requireNonEmpty(const Container & values, const std::string & name) {
  if (values.empty()) { throw std::invalid_argument(name + " must not be empty"); }
}

template<typename Range>
void requireUnique(const Range & values, const std::string & name) {
  std::set<typename Range::value_type> seen;
  for (const auto & value : values) {
    if (!seen.insert(value).second) { throw std::invalid_argument(name + " must contain unique values"); }
  }
}

/// Scalar positivity; zero is invalid. Matrix overload below additionally checks symmetry.
template<typename Scalar, std::enable_if_t<std::is_arithmetic_v<Scalar>, int> = 0>
void requirePositive(Scalar value, const std::string & name) {
  requireFinite(value, name);
  if (value <= Scalar(0)) { throw std::invalid_argument(name + " must be positive"); }
}

template<typename Scalar, std::enable_if_t<std::is_arithmetic_v<Scalar>, int> = 0>
void requireNonnegative(Scalar value, const std::string & name) {
  requireFinite(value, name);
  if (value < Scalar(0)) { throw std::invalid_argument(name + " must be nonnegative"); }
}

/// Inclusive finite bounds; equal endpoints are allowed.
template<typename Scalar>
void requireOrderedBounds(Scalar lower, Scalar upper, const std::string & name) {
  requireFinite(lower, name + " lower bound");
  requireFinite(upper, name + " upper bound");
  if (lower > upper) { throw std::invalid_argument(name + " lower bound exceeds upper bound"); }
}

template<typename Scalar>
void requireInRange(Scalar value, Scalar lower, Scalar upper, const std::string & name) {
  requireOrderedBounds(lower, upper, name);
  requireFinite(value, name);
  if (value < lower || value > upper) {
    throw std::invalid_argument(name + " is outside the allowed range");
  }
}

/// Require a proper 3x3 rotation (orthonormal columns and determinant +1).
template<typename Derived>
void requireRotationMatrix(const Eigen::MatrixBase<Derived> & matrix,
  const std::string & name, typename Derived::Scalar tolerance = 1e-8)
{
  using Scalar = typename Derived::Scalar;
  requireNonnegative(tolerance, name + " validation tolerance");
  if (matrix.rows() != 3 || matrix.cols() != 3) {
    throw std::invalid_argument(name + " must be a 3x3 rotation matrix");
  }
  Eigen::Matrix<Scalar, 3, 3> r;
  // Fixed storage keeps this check allocation-free in integration loops.
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) { r(i, j) = matrix(i, j); }
  }
  requireFinite(r, name);
  if (!(r.transpose() * r).isApprox(Eigen::Matrix<Scalar, 3, 3>::Identity(), tolerance) ||
      std::abs(r.determinant() - Scalar(1)) > tolerance) {
    throw std::invalid_argument(name + " must be a proper rotation matrix (SO(3))");
  }
}

/// Require a finite, nonempty, symmetric positive (semi)definite real matrix.
/// definite=true requires every eigenvalue > 0. Otherwise eigenvalues >= -tolerance
/// are accepted. Symmetry uses Eigen's relative comparison with the same tolerance.
/// Throws std::invalid_argument with the parameter name when validation fails.
template<typename Derived>
void requirePositive(const Eigen::MatrixBase<Derived> & matrix, bool definite,
  const std::string & name, typename Derived::Scalar tolerance = 1e-10)
{
  using Scalar = typename Derived::Scalar;
  static_assert(std::is_floating_point_v<Scalar>, "Matrix validation requires real floating-point values");
  if (!std::isfinite(tolerance) || tolerance < 0) {
    throw std::invalid_argument(name + " validation tolerance must be finite and nonnegative");
  }
  if (matrix.rows() == 0 || matrix.rows() != matrix.cols()) {
    throw std::invalid_argument(name + " must be a nonempty square matrix");
  }
  // Evaluate expressions once. Dynamic storage also allows a rectangular fixed-
  // size input to produce the exception above rather than a compile-time error.
  using DenseMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  const DenseMatrix values = matrix;
  if (!values.allFinite() || !values.isApprox(values.transpose(), tolerance)) {
    throw std::invalid_argument(name + " must be finite and symmetric");
  }
  Eigen::SelfAdjointEigenSolver<DenseMatrix> solver(values);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    throw std::invalid_argument(name + " eigenvalue decomposition failed");
  }
  const Scalar minimum = solver.eigenvalues().minCoeff();
  if (definite ? minimum <= Scalar(0) : minimum < -tolerance) {
    throw std::invalid_argument(name + (definite ? " must be positive definite" : " must be positive semidefinite"));
  }
}

template<typename Derived>
void requirePositiveDefinite(const Eigen::MatrixBase<Derived> & matrix,
  const std::string & name, typename Derived::Scalar tolerance = 1e-10)
{
  requirePositive(matrix, true, name, tolerance);
}

template<typename Derived>
void requirePositiveSemidefinite(const Eigen::MatrixBase<Derived> & matrix,
  const std::string & name, typename Derived::Scalar tolerance = 1e-10)
{
  requirePositive(matrix, false, name, tolerance);
}

}  // namespace farol2_utils
