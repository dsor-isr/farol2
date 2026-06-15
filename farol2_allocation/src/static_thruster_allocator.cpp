#include <static_thruster_allocator.hpp>

StaticThrusterAllocator::StaticThrusterAllocator(
  std::string base_frame,
  Eigen::Vector3d thrust_axis,
  std::vector<std::string> thruster_frames)
: base_frame_(std::move(base_frame)),
  thrust_axis_(std::move(thrust_axis)),
  thruster_frames_(std::move(thruster_frames))
{
  nr_thrusters_ = thruster_frames_.size();
}

bool StaticThrusterAllocator::initialize(
  tf2_ros::Buffer & tf_buffer,
  const rclcpp::Clock & clock,
  const rclcpp::Logger & logger)
{
  if (ready_) {
    return true;
  }

  if (!buildAllocationMatrix(tf_buffer, clock, logger)) {
    return false;
  }

  thrust_allocation_matrix_pseudo_inv_ =
    thrust_allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();

  ready_ = true;
  RCLCPP_INFO(
    logger,
    "Thruster allocation ready with %zu TF thruster frames using base frame '%s'.",
    nr_thrusters_,
    base_frame_.c_str());
  return true;
}

bool StaticThrusterAllocator::ready() const
{
  return ready_;
}

size_t StaticThrusterAllocator::thrusterCount() const
{
  return nr_thrusters_;
}

Eigen::VectorXd StaticThrusterAllocator::allocate(const Eigen::Vector<double, 6> & tau) const
{
  if (!ready_) {
    return {};
  }

  return thrust_allocation_matrix_pseudo_inv_ * tau;
}

bool StaticThrusterAllocator::buildAllocationMatrix(
  tf2_ros::Buffer & tf_buffer,
  const rclcpp::Clock & clock,
  const rclcpp::Logger & logger)
{
  thrust_allocation_matrix_.resize(6, nr_thrusters_);

  for (size_t i = 0; i < nr_thrusters_; ++i) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer.lookupTransform(
        base_frame_,
        thruster_frames_[i],
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger,
        clock,
        5000,
        "Waiting for transform %s -> %s: %s",
        base_frame_.c_str(),
        thruster_frames_[i].c_str(),
        ex.what());
      return false;
    }

    const auto & t = transform.transform.translation;
    const auto & q_msg = transform.transform.rotation;
    Eigen::Vector3d l(t.x, t.y, t.z);
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    Eigen::Vector3d f = q.normalized().toRotationMatrix() * thrust_axis_;

    thrust_allocation_matrix_.block<3, 1>(0, i) = f;
    thrust_allocation_matrix_.block<3, 1>(3, i) = l.cross(f);
  }

  return true;
}