#include <farol2_nav/filters/mahony.hpp>

#include <algorithm>
#include <cmath>

namespace farol2_nav
{
namespace filters
{

void MahonyFilter::configure(rclcpp::Node & node)
{
  const double blend = node.declare_parameter<double>("plugins.orientation_mahony.blend", 0.15);
  blend_ = std::clamp(blend, 0.0, 1.0);
}

void MahonyFilter::update(double dt_s, const MeasurementSnapshot & m, State & s)
{
  if (m.imu == nullptr) {
    return;
  }

  constexpr double RAD2DEG = 57.29577951308232;

  const auto & q = m.imu->orientation;
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll_meas_deg = std::atan2(sinr_cosp, cosr_cosp) * RAD2DEG;

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch_meas_deg = std::asin(std::clamp(sinp, -1.0, 1.0)) * RAD2DEG;

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw_meas_deg = std::atan2(siny_cosp, cosy_cosp) * RAD2DEG;

  // Example complementary behavior: integrate gyro, then blend against measured attitude.
  s.attitude_deg(0) += m.imu->angular_velocity.x * RAD2DEG * dt_s;
  s.attitude_deg(1) += m.imu->angular_velocity.y * RAD2DEG * dt_s;
  s.attitude_deg(2) += m.imu->angular_velocity.z * RAD2DEG * dt_s;

  s.attitude_deg(0) = (1.0 - blend_) * s.attitude_deg(0) + blend_ * roll_meas_deg;
  s.attitude_deg(1) = (1.0 - blend_) * s.attitude_deg(1) + blend_ * pitch_meas_deg;
  s.attitude_deg(2) = (1.0 - blend_) * s.attitude_deg(2) + blend_ * yaw_meas_deg;

  s.angular_velocity_dps(0) = m.imu->angular_velocity.x * RAD2DEG;
  s.angular_velocity_dps(1) = m.imu->angular_velocity.y * RAD2DEG;
  s.angular_velocity_dps(2) = m.imu->angular_velocity.z * RAD2DEG;
}

}  // namespace filters
}  // namespace farol2_nav
