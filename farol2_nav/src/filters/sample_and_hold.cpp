#include <farol2_nav/filters/sample_and_hold.hpp>

#include <farol2_utils/angles.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

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
  frame_prefix_ = node.declare_parameter<std::string>("frame_prefix", "");
  base_frame_ = frame_prefix_ + "base_link";
  clock_ = node.get_clock();
  logger_ = node.get_logger();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    node.get_node_base_interface(),
    node.get_node_logging_interface(),
    node.get_node_parameters_interface(),
    node.get_node_topics_interface());
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

bool SampleAndHoldFilter::lookup_sensor_to_base_transform(
  const std::string & sensor_frame,
  const rclcpp::Time & stamp,
  Eigen::Isometry3d & transform_base_sensor) const
{
  transform_base_sensor = Eigen::Isometry3d::Identity();

  if (sensor_frame.empty() || sensor_frame == base_frame_) {
    return true;
  }

  try {
    const auto transform_msg = tf_buffer_->lookupTransform(base_frame_, sensor_frame, stamp);
    transform_base_sensor = tf2::transformToEigen(transform_msg);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "Skipping measurement in frame '%s': cannot transform to '%s' (%s)",
      sensor_frame.c_str(),
      base_frame_.c_str(),
      ex.what());
    return false;
  }
}

void SampleAndHoldFilter::compute(double, const MeasurementSnapshot & m, State & s)
{
  if (m.imu != nullptr) {
    Eigen::Isometry3d tf_base_sensor;
    if (!lookup_sensor_to_base_transform(
        m.imu->header.frame_id,
        m.imu->header.stamp,
        tf_base_sensor)) {
      if (initialized_) {
        s = s_;
      }
      return;
    }

    mark_received("imu");
    Eigen::Quaterniond q_ns;
    tf2::fromMsg(m.imu->orientation, q_ns);
    const Eigen::Matrix3d rotation_ns = q_ns.normalized().toRotationMatrix();

    const Eigen::Matrix3d rotation_bs = tf_base_sensor.linear();
    s_.rotation_bn = rotation_ns * rotation_bs.transpose();

    const tf2::Matrix3x3 r_bn_tf(
      s_.rotation_bn(0, 0), s_.rotation_bn(0, 1), s_.rotation_bn(0, 2),
      s_.rotation_bn(1, 0), s_.rotation_bn(1, 1), s_.rotation_bn(1, 2),
      s_.rotation_bn(2, 0), s_.rotation_bn(2, 1), s_.rotation_bn(2, 2));

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    r_bn_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    s_.attitude(0) = farol2_utils::rad2deg(roll_rad);
    s_.attitude(1) = farol2_utils::rad2deg(pitch_rad);
    s_.attitude(2) = farol2_utils::rad2deg(yaw_rad);

    const Eigen::Vector3d angular_velocity_sensor(
      m.imu->angular_velocity.x,
      m.imu->angular_velocity.y,
      m.imu->angular_velocity.z);
    const Eigen::Vector3d angular_velocity_body = rotation_bs * angular_velocity_sensor;
    s_.angular_velocity(0) = farol2_utils::rad2deg(angular_velocity_body(0));
    s_.angular_velocity(1) = farol2_utils::rad2deg(angular_velocity_body(1));
    s_.angular_velocity(2) = farol2_utils::rad2deg(angular_velocity_body(2));
  }

  if (m.gnss != nullptr) {
    Eigen::Isometry3d tf_base_sensor;
    if (!lookup_sensor_to_base_transform(
        m.gnss->header.frame_id,
        m.gnss->header.stamp,
        tf_base_sensor)) {
      if (initialized_) {
        s = s_;
      }
      return;
    }

    mark_received("gnss");
    if (std::isfinite(m.gnss->latitude) && std::isfinite(m.gnss->longitude) &&
      m.gnss->latitude >= -90.0 && m.gnss->latitude <= 90.0 &&
      m.gnss->longitude >= -180.0 && m.gnss->longitude <= 180.0)
    {
      int zone = 0;
      bool northp = true;
      double easting = 0.0;
      double northing = 0.0;
      GeographicLib::UTMUPS::Forward(m.gnss->latitude, m.gnss->longitude, zone, northp, easting, northing);

      const Eigen::Vector3d sensor_position_ned(northing, easting, -m.gnss->altitude);
      const Eigen::Vector3d sensor_offset_ned =
        s_.rotation_bn * tf_base_sensor.translation();
      const Eigen::Vector3d base_position_ned = sensor_position_ned - sensor_offset_ned;

      s_.northing = base_position_ned(0);
      s_.easting = base_position_ned(1);
      s_.utm_zone = static_cast<int32_t>(zone);

      double latitude = 0.0;
      double longitude = 0.0;
      GeographicLib::UTMUPS::Reverse(zone, northp, s_.easting, s_.northing, latitude, longitude);
      s_.latitude = latitude;
      s_.longitude = longitude;
    }
  }

  if (m.utm_ned != nullptr) {
    Eigen::Isometry3d tf_base_sensor;
    if (!lookup_sensor_to_base_transform(
        m.utm_ned->header.frame_id,
        m.utm_ned->header.stamp,
        tf_base_sensor)) {
      if (initialized_) {
        s = s_;
      }
      return;
    }

    mark_received("utm_ned");
    const Eigen::Vector3d sensor_position_ned(
      m.utm_ned->vector.x,
      m.utm_ned->vector.y,
      s_.depth);
    const Eigen::Vector3d sensor_offset_ned =
      s_.rotation_bn * tf_base_sensor.translation();
    const Eigen::Vector3d base_position_ned = sensor_position_ned - sensor_offset_ned;
    s_.northing = base_position_ned(0);
    s_.easting = base_position_ned(1);
    s_.utm_zone = static_cast<int32_t>(m.utm_ned->vector.z);
  }

  if (m.velocity_over_ground != nullptr) {
    Eigen::Isometry3d tf_base_sensor;
    if (!lookup_sensor_to_base_transform(
        m.velocity_over_ground->header.frame_id,
        m.velocity_over_ground->header.stamp,
        tf_base_sensor)) {
      if (initialized_) {
        s = s_;
      }
      return;
    }

    mark_received("velocity_over_ground");
    const Eigen::Matrix3d rotation_bs = tf_base_sensor.linear();
    const Eigen::Vector3d sensor_offset_body = tf_base_sensor.translation();
    const Eigen::Vector3d angular_velocity_body = farol2_utils::deg2rad(1.0) * s_.angular_velocity;
    const Eigen::Vector3d velocity_sensor(
      m.velocity_over_ground->vector.x,
      m.velocity_over_ground->vector.y,
      m.velocity_over_ground->vector.z);
    s_.velocity_over_ground_body =
      rotation_bs * velocity_sensor - angular_velocity_body.cross(sensor_offset_body);
    s_.velocity_over_ground_ned = s_.rotation_bn * s_.velocity_over_ground_body;
  }

  if (m.velocity_through_water != nullptr) {
    Eigen::Isometry3d tf_base_sensor;
    if (!lookup_sensor_to_base_transform(
        m.velocity_through_water->header.frame_id,
        m.velocity_through_water->header.stamp,
        tf_base_sensor)) {
      if (initialized_) {
        s = s_;
      }
      return;
    }

    mark_received("velocity_through_water");
    const Eigen::Matrix3d rotation_bs = tf_base_sensor.linear();
    const Eigen::Vector3d sensor_offset_body = tf_base_sensor.translation();
    const Eigen::Vector3d angular_velocity_body = farol2_utils::deg2rad(1.0) * s_.angular_velocity;
    const Eigen::Vector3d velocity_sensor(
      m.velocity_through_water->vector.x,
      m.velocity_through_water->vector.y,
      m.velocity_through_water->vector.z);
    s_.velocity_through_water_body =
      rotation_bs * velocity_sensor - angular_velocity_body.cross(sensor_offset_body);
    s_.velocity_through_water_ned = s_.rotation_bn * s_.velocity_through_water_body;
  }

  if (m.depth != nullptr) {
    Eigen::Isometry3d tf_base_sensor;
    if (!lookup_sensor_to_base_transform(
        m.depth->header.frame_id,
        m.depth->header.stamp,
        tf_base_sensor)) {
      if (initialized_) {
        s = s_;
      }
      return;
    }

    mark_received("depth");
    const Eigen::Vector3d sensor_offset_ned =
      s_.rotation_bn * tf_base_sensor.translation();
    s_.depth = m.depth->depth - sensor_offset_ned(2);
  }

  if (m.altimeter != nullptr) {
    mark_received("altimeter");
    s_.altimeter = m.altimeter->range;
  }

  if (m.control_surface_deflection != nullptr) {
    mark_received("control_surface_deflection");
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
