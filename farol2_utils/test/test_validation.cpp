#include <gtest/gtest.h>
#include <limits>
#include <vector>
#include "farol2_utils/validation.hpp"

using namespace farol2_utils;

TEST(MatrixValidation, PositiveDefiniteAndExpressions) {
  Eigen::Matrix3d a;
  a << 2.0, 0.1, 0.0, 0.1, 3.0, 0.2, 0.0, 0.2, 4.0;
  EXPECT_NO_THROW(requirePositiveDefinite(a, "inertia"));
  EXPECT_NO_THROW(requirePositiveDefinite(a.transpose() * a, "mass"));
  EXPECT_NO_THROW(requirePositiveSemidefinite(a, "covariance"));
}

TEST(MatrixValidation, SingularAndZeroMatricesAreOnlySemidefinite) {
  Eigen::Matrix3d a = Eigen::Matrix3d::Zero();
  EXPECT_NO_THROW(requirePositiveSemidefinite(a, "damping"));
  EXPECT_THROW(requirePositiveDefinite(a, "inertia"), std::invalid_argument);
  a(0, 0) = 1.0;
  EXPECT_NO_THROW(requirePositiveSemidefinite(a, "added mass"));
  EXPECT_THROW(requirePositiveDefinite(a, "total mass"), std::invalid_argument);
}

TEST(MatrixValidation, NegativeEigenvaluesAreRejected) {
  Eigen::Matrix2d a = Eigen::Matrix2d::Identity();
  a(0, 0) = -1.0;
  EXPECT_THROW(requirePositiveDefinite(a, "inertia"), std::invalid_argument);
  EXPECT_THROW(requirePositiveSemidefinite(a, "damping"), std::invalid_argument);
}

TEST(MatrixValidation, RejectsNonsymmetricAndNonfiniteInputs) {
  Eigen::Matrix2d a = Eigen::Matrix2d::Identity();
  a(0, 1) = 0.5;
  EXPECT_THROW(requirePositiveDefinite(a, "mass"), std::invalid_argument);
  a.setIdentity(); a(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(requirePositiveSemidefinite(a, "covariance"), std::invalid_argument);
  a(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(requirePositiveDefinite(a, "inertia"), std::invalid_argument);
}

TEST(MatrixValidation, RejectsEmptyAndRectangularMatrices) {
  EXPECT_THROW(requirePositiveDefinite(Eigen::MatrixXd(0, 0), "mass"), std::invalid_argument);
  EXPECT_THROW(requirePositiveSemidefinite(Eigen::MatrixXd::Zero(2, 3), "damping"), std::invalid_argument);
  const Eigen::Matrix<double, 2, 3> fixed = Eigen::Matrix<double, 2, 3>::Zero();
  EXPECT_THROW(requirePositiveDefinite(fixed, "inertia"), std::invalid_argument);
}

TEST(MatrixValidation, SemidefiniteToleranceIsConfigurable) {
  Eigen::Matrix2d a = Eigen::Matrix2d::Identity();
  a(0, 0) = -5e-11;
  EXPECT_NO_THROW(requirePositiveSemidefinite(a, "covariance"));
  EXPECT_THROW(requirePositiveSemidefinite(a, "covariance", 0.0), std::invalid_argument);
  EXPECT_THROW(requirePositiveDefinite(a, "inertia"), std::invalid_argument);
  EXPECT_THROW(requirePositiveSemidefinite(a, "covariance", -1.0), std::invalid_argument);
}

TEST(MatrixValidation, ErrorNamesTheParameterAndSupportsFloat) {
  EXPECT_NO_THROW(requirePositiveDefinite(Eigen::Matrix2f::Identity(), "gain"));
  try {
    requirePositiveDefinite(Eigen::Matrix2d::Zero(), "vehicle.inertia");
    FAIL() << "Expected invalid matrix to be rejected";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("vehicle.inertia"), std::string::npos);
  }
}

TEST(ScalarValidation, PositiveNonnegativeAndFiniteHaveDistinctBoundaries) {
  EXPECT_NO_THROW(requirePositive(2, "frequency"));
  EXPECT_NO_THROW(requireNonnegative(0.0, "delay"));
  EXPECT_NO_THROW(requireFinite(-1.0, "offset"));
  EXPECT_THROW(requirePositive(0.0, "dt"), std::invalid_argument);
  EXPECT_THROW(requireNonnegative(-0.1, "pole"), std::invalid_argument);
  EXPECT_THROW(requirePositive(std::numeric_limits<double>::infinity(), "mass"), std::invalid_argument);
  EXPECT_THROW(requireFinite(std::numeric_limits<double>::quiet_NaN(), "command"), std::invalid_argument);
}

TEST(ScalarValidation, InclusiveRangeAndOrderedBounds) {
  EXPECT_NO_THROW(requireInRange(-90.0, -90.0, 90.0, "latitude"));
  EXPECT_NO_THROW(requireInRange(90.0, -90.0, 90.0, "latitude"));
  EXPECT_NO_THROW(requireOrderedBounds(0.0, 0.0, "constant current"));
  EXPECT_THROW(requireInRange(90.1, -90.0, 90.0, "latitude"), std::invalid_argument);
  EXPECT_THROW(requireOrderedBounds(1.0, -1.0, "limits"), std::invalid_argument);
  EXPECT_THROW(requireInRange(1.0, 0.0, std::numeric_limits<double>::infinity(), "range"), std::invalid_argument);
}

TEST(ArrayValidation, SizesAndFiniteEntries) {
  std::vector<double> values{1.0, 2.0, 3.0};
  EXPECT_NO_THROW(requireSize(values, 3, "position"));
  EXPECT_NO_THROW(requireFiniteValues(values, "position"));
  EXPECT_THROW(requireSize(values, 2, "position"), std::invalid_argument);
  values[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(requireFiniteValues(values, "position"), std::invalid_argument);
  Eigen::Vector3d v(1.0, 2.0, std::numeric_limits<double>::infinity());
  EXPECT_THROW(requireFinite(v, "velocity"), std::invalid_argument);
}

TEST(ArrayValidation, EmptyAndDuplicateContainers) {
  const std::vector<int> indices{0, 1, 5}, duplicate{0, 1, 0}, empty;
  EXPECT_NO_THROW(requireNonEmpty(indices, "indices"));
  EXPECT_NO_THROW(requireUnique(indices, "indices"));
  EXPECT_THROW(requireNonEmpty(empty, "indices"), std::invalid_argument);
  EXPECT_THROW(requireNonEmpty(std::string{}, "frame"), std::invalid_argument);
  EXPECT_THROW(requireUnique(duplicate, "indices"), std::invalid_argument);
  EXPECT_THROW(requireUnique(std::vector<std::string>{"left", "left"}, "frames"), std::invalid_argument);
}

TEST(RotationValidation, RejectsScaleReflectionWrongShapeAndNonfiniteValues) {
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  EXPECT_NO_THROW(requireRotationMatrix(r, "attitude"));
  r(0, 0) = -1.0;  // Orthogonal, but a reflection is not a rotation.
  EXPECT_THROW(requireRotationMatrix(r, "attitude"), std::invalid_argument);
  r(0, 0) = 2.0;
  EXPECT_THROW(requireRotationMatrix(r, "attitude"), std::invalid_argument);
  r(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(requireRotationMatrix(r, "attitude"), std::invalid_argument);
  EXPECT_THROW(requireRotationMatrix(Eigen::Matrix2d::Identity(), "attitude"), std::invalid_argument);
}
