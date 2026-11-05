#include <static_thruster_allocator.hpp>
#include <thruster_geometry.hpp>

StaticThrusterAllocator::StaticThrusterAllocator(
  std::string base_frame,
  std::vector<std::string> thruster_frames)
: base_frame_(std::move(base_frame)),
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

  // The pseudo-inverse provides the least-squares force split for any thruster layout.
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
    // Avoid using an uninitialised matrix while the node is still waiting for TF.
    return {};
  }

  return thrust_allocation_matrix_pseudo_inv_ * tau;
}

bool StaticThrusterAllocator::buildAllocationMatrix(
  tf2_ros::Buffer & tf_buffer,
  const rclcpp::Clock & clock,
  const rclcpp::Logger & logger)
{
  std::vector<ThrusterGeometry> geometry;
  if (!buildThrusterGeometryFromTF(
      tf_buffer,
      clock,
      logger,
      base_frame_,
      thruster_frames_,
      geometry)) {
    return false;
  }

  thrust_allocation_matrix_ = buildThrustAllocationMatrix(geometry);
  return true;
}
