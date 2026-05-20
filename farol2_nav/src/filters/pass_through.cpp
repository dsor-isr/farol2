#include <farol2_nav/filters/pass_through.hpp>

#include <algorithm>
#include <GeographicLib/UTMUPS.hpp>

namespace farol2_nav
{
namespace filters
{

void PassThroughFilter::configure(rclcpp::Node &)
{
}

void PassThroughFilter::update(double, const MeasurementSnapshot & m, State & s)
{
  if (m.gnss != nullptr) {
    s.latitude_deg = m.gnss->latitude;
    s.longitude_deg = m.gnss->longitude;

    if (std::isfinite(s.latitude_deg) && std::isfinite(s.longitude_deg) &&
      s.latitude_deg >= -90.0 && s.latitude_deg <= 90.0 &&
      s.longitude_deg >= -180.0 && s.longitude_deg <= 180.0)
    {
      int zone = 0;
      bool northp = true;
      double easting = 0.0;
      double northing = 0.0;
      GeographicLib::UTMUPS::Forward(s.latitude_deg, s.longitude_deg, zone, northp, easting, northing);
      s.northing_m = northing;
      s.easting_m = easting;
      s.utm_zone = static_cast<int32_t>(zone);
    }
  }

  if (m.utm_ned != nullptr) {
    s.northing_m = m.utm_ned->vector.x;
    s.easting_m = m.utm_ned->vector.y;
    s.utm_zone = static_cast<int32_t>(m.utm_ned->vector.z);
  }

  if (m.velocity_over_ground != nullptr) {
    s.velocity_over_ground_ned_mps <<
      m.velocity_over_ground->vector.x,
      m.velocity_over_ground->vector.y,
      m.velocity_over_ground->vector.z;
  }

  if (m.velocity_through_water != nullptr) {
    s.velocity_through_water_body_mps <<
      m.velocity_through_water->vector.x,
      m.velocity_through_water->vector.y,
      m.velocity_through_water->vector.z;
  }

  if (m.depth != nullptr) {
    s.depth_m = m.depth->data;
  }

  if (m.altimeter != nullptr) {
    s.altimeter_m = m.altimeter->data;
  }

  if (m.imu != nullptr) {
    const auto & q = m.imu->orientation;
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll_rad = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    const double pitch_rad = std::asin(std::clamp(sinp, -1.0, 1.0));

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw_rad = std::atan2(siny_cosp, cosy_cosp);

    constexpr double RAD2DEG = 57.29577951308232;
    s.attitude_deg(0) = roll_rad * RAD2DEG;
    s.attitude_deg(1) = pitch_rad * RAD2DEG;
    s.attitude_deg(2) = yaw_rad * RAD2DEG;

    s.angular_velocity_dps(0) = m.imu->angular_velocity.x * RAD2DEG;
    s.angular_velocity_dps(1) = m.imu->angular_velocity.y * RAD2DEG;
    s.angular_velocity_dps(2) = m.imu->angular_velocity.z * RAD2DEG;
  }
}

}  // namespace filters
}  // namespace farol2_nav
