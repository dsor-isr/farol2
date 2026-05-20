#pragma once

#include <Eigen/Dense>

#include <cstdint>

namespace farol2_nav
{
namespace filters
{

// Internal state mirrors NavigationState fields to keep mapping direct and cheap.
struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Position
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double northing_m{0.0};
  double easting_m{0.0};
  int32_t utm_zone{0};

  // Vertical information
  double depth_m{0.0};
  double altimeter_m{0.0};
  double altitude_ellipsoidal_m{0.0};
  double local_datum_altitude_m{0.0};

  // Velocities
  Eigen::Vector3d velocity_over_ground_body_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_through_water_body_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_over_ground_ned_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_through_water_ned_mps{Eigen::Vector3d::Zero()};
  double course_over_ground_deg{0.0};
  Eigen::Vector3d current_velocity_inertial_mps{Eigen::Vector3d::Zero()};

  // Attitude
  Eigen::Vector3d attitude_deg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_dps{Eigen::Vector3d::Zero()};
};

}  // namespace filters
}  // namespace farol2_nav
