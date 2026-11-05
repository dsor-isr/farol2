#include "farol2_sim/sensors.hpp"
#include "farol2_sim/kinematics.hpp"
#include "farol2_utils/validation.hpp"
#include <GeographicLib/UTMUPS.hpp>

namespace farol2_sim {

GnssSensor::GnssSensor(rclcpp::Node & node)
: MountedSensor(node, "gnss", "gnss_link"), noise_(node, "gnss", 0), position_noise_(noise_.load(node, "position_noise")),
  velocity_noise_(noise_.load(node, "velocity_noise")) {
  const double latitude = node.declare_parameter<double>("environment.origin.latitude");
  const double longitude = node.declare_parameter<double>("environment.origin.longitude");
  origin_altitude_ = node.declare_parameter<double>("environment.origin.altitude");
  farol2_utils::requireInRange(latitude, -90.0, 90.0, "environment.origin.latitude");
  farol2_utils::requireInRange(longitude, -180.0, 180.0, "environment.origin.longitude");
  farol2_utils::requireFinite(origin_altitude_, "environment.origin.altitude");
  GeographicLib::UTMUPS::Forward(latitude, longitude, utm_zone_, northp_, easting_, northing_);
  fix_pub_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("gnss", 1);
  if (node.declare_parameter<bool>("sensor.gnss.measure_utm_ned", false)) {
    utm_pub_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>("ned_utm", 1);
  }
  if (node.declare_parameter<bool>("sensor.gnss.measure_velocity_over_ground")) {
    velocity_pub_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>("velocity_over_ground", 1);
  }
}

void GnssSensor::publish(const SensorState & sample) {
  const auto position = sensorPosition(sample.state, mount_);
  const double north = position.x() + northing_ + noise_.sample(position_noise_, 0);
  const double east = position.y() + easting_ + noise_.sample(position_noise_, 1);
  if (utm_pub_) {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = sample.stamp;
    message.header.frame_id = frame_;
    message.vector.x = north;
    message.vector.y = east;
    message.vector.z = static_cast<double>(utm_zone_);
    utm_pub_->publish(message);
  }
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = sample.stamp;
  fix.header.frame_id = frame_;
  GeographicLib::UTMUPS::Reverse(utm_zone_, northp_, east, north, fix.latitude, fix.longitude);
  fix.altitude = origin_altitude_ - position.z();
  fix_pub_->publish(fix);
  if (velocity_pub_) {
    const auto velocity = sensorVelocity(sample.state, sample.environment, mount_, VelocityReference::Ground);
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = sample.stamp;
    message.header.frame_id = frame_;
    message.vector.x = velocity.x() + noise_.sample(velocity_noise_, 0);
    message.vector.y = velocity.y() + noise_.sample(velocity_noise_, 1);
    message.vector.z = velocity.z() + noise_.sample(velocity_noise_, 2);
    velocity_pub_->publish(message);
  }
}

}  // namespace farol2_sim
