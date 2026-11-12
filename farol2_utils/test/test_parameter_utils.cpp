#include <gtest/gtest.h>
#include <rclcpp/parameter.hpp>
#include <limits>
#include "farol2_utils/parameter_utils.hpp"

using namespace farol2_utils;

TEST(ParameterUtils, ReadsVectorAndVector3WithoutRosContext) {
  const rclcpp::Parameter p("position", std::vector<double>{1.0, 2.0, 3.0});
  EXPECT_EQ(vectorParameter(p, 3), (std::vector<double>{1.0, 2.0, 3.0}));
  EXPECT_TRUE(vector3Parameter(p).isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(ParameterUtils, RejectsWrongTypesAndVectorSizes) {
  EXPECT_THROW(vectorParameter(rclcpp::Parameter("gain", 1.0), 1), std::invalid_argument);
  EXPECT_THROW(vectorParameter(rclcpp::Parameter("gain", std::vector<int64_t>{1, 2}), 2), std::invalid_argument);
  EXPECT_THROW(vector3Parameter(rclcpp::Parameter("position", std::vector<double>{1.0, 2.0})), std::invalid_argument);
}

TEST(ParameterUtils, BuildsDiagonalAndRowMajorMatrices) {
  const auto diagonal = matrixParameter(rclcpp::Parameter("inertia", std::vector<double>{2.0, 3.0, 4.0}), 3);
  EXPECT_TRUE(diagonal.isApprox(Eigen::Vector3d(2.0, 3.0, 4.0).asDiagonal().toDenseMatrix()));
  const auto full = matrixParameter(rclcpp::Parameter("matrix", std::vector<double>{1.0, 2.0, 3.0, 4.0}), 2);
  EXPECT_DOUBLE_EQ(full(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(full(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(full(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(full(1, 1), 4.0);
  std::vector<double> values(36);
  for (size_t i = 0; i < values.size(); ++i) { values[i] = static_cast<double>(i); }
  const auto six = matrixParameter(rclcpp::Parameter("added_mass", values), 6);
  EXPECT_DOUBLE_EQ(six(2, 4), 16.0);
  EXPECT_DOUBLE_EQ(six(5, 5), 35.0);
}

TEST(ParameterUtils, RejectsInvalidMatrixShapesAndDimensions) {
  const rclcpp::Parameter p("mass", std::vector<double>{1.0, 2.0, 3.0});
  EXPECT_THROW(matrixParameter(p, 2), std::invalid_argument);
  EXPECT_THROW(matrixParameter(p, 0), std::invalid_argument);
  EXPECT_THROW(matrixParameter(p, -1), std::invalid_argument);
}

TEST(ParameterUtils, RejectsNonfiniteArraysAndNamesParameter) {
  const rclcpp::Parameter p("vehicle.inertia", std::vector<double>{1.0, std::numeric_limits<double>::quiet_NaN(), 3.0});
  EXPECT_THROW(vector3Parameter(p), std::invalid_argument);
  try {
    matrixParameter(p, 3);
    FAIL() << "Expected nonfinite parameter to be rejected";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("vehicle.inertia"), std::string::npos);
  }
}
