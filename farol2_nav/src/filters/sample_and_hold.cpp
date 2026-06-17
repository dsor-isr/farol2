#include <farol2_nav/filters/sample_and_hold.hpp>

#include <farol2_utils/angles.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <GeographicLib/UTMUPS.hpp>

namespace farol2_nav
{
namespace filters
{

void SampleAndHoldFilter::configure(rclcpp::Node & node)
{
  node.get_parameter("measurements", required_measurements_);
  received_measurements_.clear();
  initialized_ = required_measurements_.empty();
}

bool SampleAndHoldFilter::has_measurement(const std::string & name) const
{
  return std::find(required_measurements_.begin(), required_measurements_.end(), name) !=
    required_measurements_.end();
}

void SampleAndHoldFilter::mark_received(const std::string & name)
{
  if (!has_measurement(name)) {
    return;
  }

  const auto it = std::find(received_measurements_.begin(), received_measurements_.end(), name);
  if (it == received_measurements_.end()) {
    received_measurements_.push_back(name);
  }
}

bool SampleAndHoldFilter::all_required_measurements_received() const
{
  return received_measurements_.size() >= required_measurements_.size();
}

void SampleAndHoldFilter::compute(double, const MeasurementSnapshot & m, State & s)
{
  if (m.imu != nullptr) {
    mark_received("imu");
    tf2::Quaternion q_tf;
    tf2::fromMsg(m.imu->orientation, q_tf);

    const tf2::Matrix3x3 r_bn_tf(q_tf);
    s_.rotation_bn <<
      r_bn_tf[0][0], r_bn_tf[0][1], r_bn_tf[0][2],
      r_bn_tf[1][0], r_bn_tf[1][1], r_bn_tf[1][2],
      r_bn_tf[2][0], r_bn_tf[2][1], r_bn_tf[2][2];

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    r_bn_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    s_.attitude(0) = farol2_utils::rad2deg(roll_rad);
    s_.attitude(1) = farol2_utils::rad2deg(pitch_rad);
    s_.attitude(2) = farol2_utils::rad2deg(yaw_rad);

    s_.angular_velocity(0) = farol2_utils::rad2deg(m.imu->angular_velocity.x);
    s_.angular_velocity(1) = farol2_utils::rad2deg(m.imu->angular_velocity.y);
    s_.angular_velocity(2) = farol2_utils::rad2deg(m.imu->angular_velocity.z);
  }

  if (m.gnss != nullptr) {
    mark_received("gnss");
    s_.latitude = m.gnss->latitude;
    s_.longitude = m.gnss->longitude;

    if (std::isfinite(s_.latitude) && std::isfinite(s_.longitude) &&
      s_.latitude >= -90.0 && s_.latitude <= 90.0 &&
      s_.longitude >= -180.0 && s_.longitude <= 180.0)
    {
      int zone = 0;
      bool northp = true;
      double easting = 0.0;
      double northing = 0.0;
      GeographicLib::UTMUPS::Forward(s_.latitude, s_.longitude, zone, northp, easting, northing);
      s_.northing = northing;
      s_.easting = easting;
      s_.utm_zone = static_cast<int32_t>(zone);
    }
  }

  if (m.utm_ned != nullptr) {
    mark_received("utm_ned");
    s_.northing = m.utm_ned->vector.x;
    s_.easting = m.utm_ned->vector.y;
    s_.utm_zone = static_cast<int32_t>(m.utm_ned->vector.z);
  }

  if (m.velocity_over_ground != nullptr) {
    mark_received("velocity_over_ground");
    s_.velocity_over_ground_ned <<
      m.velocity_over_ground->vector.x,
      m.velocity_over_ground->vector.y,
      m.velocity_over_ground->vector.z;
    s_.velocity_over_ground_body = s_.rotation_bn.transpose() * s_.velocity_over_ground_ned;
  }

  if (m.velocity_through_water != nullptr) {
    mark_received("velocity_through_water");
    s_.velocity_through_water_ned <<
      m.velocity_through_water->vector.x,
      m.velocity_through_water->vector.y,
      m.velocity_through_water->vector.z;
    s_.velocity_through_water_body = s_.rotation_bn.transpose() * s_.velocity_through_water_ned;
  }

  if (m.depth != nullptr) {
    mark_received("depth");
    s_.depth = m.depth->data;
  }

  if (m.altimeter != nullptr) {
    mark_received("altimeter");
    s_.altimeter = m.altimeter->data;
  }

  if (m.rudder_angle != nullptr) {
    mark_received("rudder_angle");
  }

  if (m.thruster_rpm != nullptr) {
    mark_received("thruster_rpm");
  }

  if (!initialized_ && all_required_measurements_received()) {
    initialized_ = true;
  }

  if (initialized_) {
    s = s_;
  }
}

}  // namespace filters
}  // namespace farol2_nav
