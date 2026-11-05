#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

namespace farol2_nav
{
namespace filters
{

// Internal state mirrors NavigationState fields to keep mapping direct and cheap.
struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Position
  double latitude{0.0};
  double longitude{0.0};
  double northing{0.0};
  double easting{0.0};
  int32_t utm_zone{0};

  // Vertical information
  double depth{0.0};
  double altimeter{0.0};
  double altitude_wgs84{0.0};
  double altitude_local_datum_altitude{0.0};

  // Velocities
  Eigen::Vector3d velocity_over_ground_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_through_water_body{Eigen::Vector3d::Zero()};
  double course_over_ground{0.0};
  Eigen::Vector3d current_velocity_ned{Eigen::Vector3d::Zero()};

  // Attitude as roll/pitch/yaw [deg] for convenience.
  Eigen::Vector3d attitude{Eigen::Vector3d::Zero()};
  // Rotation from body frame to NED frame.
  Eigen::Matrix3d rotation_bn{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};

  // Actuator states
  std::vector<double> thruster_rpm{};
  std::vector<double> control_surface_angle{};
};

}  // namespace filters
}  // namespace farol2_nav
