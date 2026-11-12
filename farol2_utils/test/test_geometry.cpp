#include <gtest/gtest.h>
#include "farol2_utils/geometry.hpp"

using namespace farol2_utils;

TEST(Geometry, SkewMatchesCrossProduct) {
  const Eigen::Vector3d a(1.0, -2.0, 3.0), b(4.0, 5.0, -6.0);
  EXPECT_TRUE((skew(a) * b).isApprox(a.cross(b)));
  EXPECT_TRUE(skew(a).isApprox(-skew(a).transpose()));
}

TEST(Geometry, RotationExponentialHandlesZeroAndSmallAngles) {
  EXPECT_TRUE(rotationIncrement(Eigen::Vector3d::Zero()).isIdentity());
  for (const double scale : {1e-12, 1e-9, 0.1, 2.0}) {
    const Eigen::Vector3d vector = scale * Eigen::Vector3d(0.2, -0.3, 0.4);
    const Eigen::Matrix3d r = rotationIncrement(vector);
    EXPECT_TRUE((r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
    EXPECT_TRUE(r.isApprox(Eigen::AngleAxisd(vector.norm(), vector.normalized()).toRotationMatrix(), 1e-12));
  }
}

TEST(Geometry, RpyConversionsRoundTripRotation) {
  for (const auto & angles : {Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(-0.8, 1.2, -2.4)}) {
    const Eigen::Matrix3d r = rotationFromRpy(angles);
    EXPECT_TRUE(rotationFromRpy(rpyFromRotation(r)).isApprox(r, 1e-12));
  }
}

TEST(Geometry, PointVelocityIncludesAngularLeverArm) {
  const Eigen::Vector3d origin_velocity(2.0, 0.5, 0.0), omega(0.0, 0.0, 0.25);
  const Eigen::Vector3d offset(-4.0, 1.5, 0.0);
  EXPECT_TRUE(pointVelocity(origin_velocity, omega, offset).isApprox(Eigen::Vector3d(1.625, -0.5, 0.0)));
  EXPECT_TRUE(pointVelocity(origin_velocity, omega, Eigen::Vector3d::Zero()).isApprox(origin_velocity));
}

TEST(Geometry, WrenchRotatesForceAndUsesBodyMomentArm) {
  const Eigen::Matrix3d rotation = rotationFromRpy(Eigen::Vector3d(0.0, 0.0, std::acos(-1.0) / 2.0));
  const auto wrench = forceToWrench(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0), rotation);
  EXPECT_TRUE(wrench.head<3>().isApprox(Eigen::Vector3d(0.0, 10.0, 0.0), 1e-12));
  EXPECT_TRUE(wrench.tail<3>().isApprox(Eigen::Vector3d(0.0, 0.0, 20.0), 1e-12));
}
