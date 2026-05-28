#include <farol2_nav/filters/sample_and_hold.hpp>

#include <farol2_utils/angles.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <GeographicLib/UTMUPS.hpp>

namespace farol2_nav
{
namespace filters
{

void SampleAndHoldFilter::configure(rclcpp::Node &)
{
}

void SampleAndHoldFilter::compute(double, const MeasurementSnapshot & m, State & s)
{
  if (m.gnss != nullptr) {
    s.latitude = m.gnss->latitude;
    s.longitude = m.gnss->longitude;

    if (std::isfinite(s.latitude) && std::isfinite(s.longitude) &&
      s.latitude >= -90.0 && s.latitude <= 90.0 &&
      s.longitude >= -180.0 && s.longitude <= 180.0)
    {
      int zone = 0;
      bool northp = true;
      double easting = 0.0;
      double northing = 0.0;
      GeographicLib::UTMUPS::Forward(s.latitude, s.longitude, zone, northp, easting, northing);
      s.northing = northing;
      s.easting = easting;
      s.utm_zone = static_cast<int32_t>(zone);
    }
  }

  if (m.utm_ned != nullptr) {
    s.northing = m.utm_ned->vector.x;
    s.easting = m.utm_ned->vector.y;
    s.utm_zone = static_cast<int32_t>(m.utm_ned->vector.z);
  }

  if (m.velocity_over_ground != nullptr) {
    s.velocity_over_ground_ned <<
      m.velocity_over_ground->vector.x,
      m.velocity_over_ground->vector.y,
      m.velocity_over_ground->vector.z;
  }

  if (m.velocity_through_water != nullptr) {
    s.velocity_through_water_body <<
      m.velocity_through_water->vector.x,
      m.velocity_through_water->vector.y,
      m.velocity_through_water->vector.z;
  }

  if (m.depth != nullptr) {
    s.depth = m.depth->data;
  }

  if (m.altimeter != nullptr) {
    s.altimeter = m.altimeter->data;
  }

  if (m.imu != nullptr) {
    tf2::Quaternion q_tf;
    tf2::fromMsg(m.imu->orientation, q_tf);

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    tf2::Matrix3x3(q_tf).getRPY(roll_rad, pitch_rad, yaw_rad);

    s.attitude(0) = farol2_utils::rad2deg(roll_rad);
    s.attitude(1) = farol2_utils::rad2deg(pitch_rad);
    s.attitude(2) = farol2_utils::rad2deg(yaw_rad);

    s.angular_velocity(0) = farol2_utils::rad2deg(m.imu->angular_velocity.x);
    s.angular_velocity(1) = farol2_utils::rad2deg(m.imu->angular_velocity.y);
    s.angular_velocity(2) = farol2_utils::rad2deg(m.imu->angular_velocity.z);
  }
}

}  // namespace filters
}  // namespace farol2_nav
