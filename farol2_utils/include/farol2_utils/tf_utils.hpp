#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace farol2_utils
{

class StaticTransformLookup
{
public:
  explicit StaticTransformLookup(rclcpp::Node & node);

  const Eigen::Isometry3d & lookup(
    const std::string & target_frame,
    const std::string & source_frame);

  void clear();

private:
  std::string cache_key(
    const std::string & target_frame,
    const std::string & source_frame) const;

  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, Eigen::Isometry3d> transforms_;
  std::unordered_set<std::string> warned_missing_transforms_;
};

}  // namespace farol2_utils
