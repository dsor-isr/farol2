#include "farol2_sim/sensors.hpp"
#include "farol2_utils/validation.hpp"
#include <Eigen/Geometry>
#include <tf2/time.h>

namespace farol2_sim {

MountedSensor::MountedSensor(rclcpp::Node & node, const std::string & name,
  const std::string & default_frame) {
  frame_ = node.declare_parameter<std::string>("sensor." + name + ".frame", default_frame);
  farol2_utils::requireNonEmpty(frame_, "sensor." + name + ".frame");
  const auto prefix = node.get_parameter("frame_prefix").as_string();
  if (frame_.rfind(prefix, 0) != 0) { frame_ = prefix + frame_; }
}

void MountedSensor::initializeGeometry(tf2_ros::Buffer & buffer, const std::string & base_frame) {
  if (geometry_ready_) { return; }
  const auto transform = buffer.lookupTransform(base_frame, frame_, tf2::TimePointZero);
  const auto & t = transform.transform.translation;
  const auto & r = transform.transform.rotation;
  const Eigen::Quaterniond rotation(r.w, r.x, r.y, r.z);
  farol2_utils::requireFinite(rotation.coeffs(), "sensor mounting quaternion: " + frame_);
  if (rotation.norm() < 1e-9) {
    throw std::invalid_argument("Invalid sensor mounting quaternion: " + frame_);
  }
  mount_.position_body = Eigen::Vector3d(t.x, t.y, t.z);
  mount_.rotation_to_body = rotation.normalized().toRotationMatrix();
  farol2_utils::requireFinite(mount_.position_body, "sensor mounting position: " + frame_);
  geometry_ready_ = true;
}

}  // namespace farol2_sim
