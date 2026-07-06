#include <farol2_utils/tf_utils.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace farol2_utils
{

StaticTransformLookup::StaticTransformLookup(rclcpp::Node & node)
: logger_(node.get_logger()),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node.get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    node.get_node_base_interface(),
    node.get_node_logging_interface(),
    node.get_node_parameters_interface(),
    node.get_node_topics_interface()))
{
}

const Eigen::Isometry3d & StaticTransformLookup::lookup(
  const std::string & target_frame,
  const std::string & source_frame)
{
  static const Eigen::Isometry3d identity_transform = Eigen::Isometry3d::Identity();

  const auto key = cache_key(target_frame, source_frame);
  const auto cached_transform = transforms_.find(key);
  if (cached_transform != transforms_.end()) {
    return cached_transform->second;
  }

  if (target_frame.empty() || source_frame.empty() || target_frame == source_frame) {
    return identity_transform;
  }

  try {
    const auto transform_msg =
      tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    const auto inserted_transform =
      transforms_.emplace(key, tf2::transformToEigen(transform_msg));
    warned_missing_transforms_.erase(key);
    return inserted_transform.first->second;
  } catch (const tf2::TransformException & ex) {
    if (warned_missing_transforms_.insert(key).second) {
      RCLCPP_WARN(
        logger_,
        "Static transform '%s' -> '%s' is not available, using identity: %s",
        source_frame.c_str(),
        target_frame.c_str(),
        ex.what());
    }
    return identity_transform;
  }
}

void StaticTransformLookup::clear()
{
  transforms_.clear();
  warned_missing_transforms_.clear();
}

std::string StaticTransformLookup::cache_key(
  const std::string & target_frame,
  const std::string & source_frame) const
{
  return target_frame + "<-" + source_frame;
}

}  // namespace farol2_utils
