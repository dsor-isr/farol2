#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

class StaticThrusterAllocator {
  public:
    /**
     * @brief Create an allocator for the given body and thruster TF frames.
     */
    StaticThrusterAllocator(
      std::string base_frame,
      std::vector<std::string> thruster_frames);

    /**
     * @brief Build the allocation matrix when the required transforms are available.
     */
    bool initialize(tf2_ros::Buffer & tf_buffer, const rclcpp::Clock & clock, const rclcpp::Logger & logger);

    /**
     * @brief Report whether the allocation matrix is ready for use.
     */
    bool ready() const;

    /**
     * @brief Return the number of configured thrusters.
     */
    size_t thrusterCount() const;

    /**
     * @brief Map a requested body wrench to individual thruster forces.
     */
    Eigen::VectorXd allocate(const Eigen::Vector<double, 6> & tau) const;

  private:
    /**
     * @brief Construct the thrust allocation matrix from TF geometry.
     */
    bool buildAllocationMatrix(
      tf2_ros::Buffer & tf_buffer,
      const rclcpp::Clock & clock,
      const rclcpp::Logger & logger);

    std::string base_frame_;  ///< Body frame used to express thruster geometry.
    std::vector<std::string> thruster_frames_;  ///< TF frame of each thruster.
    size_t nr_thrusters_{0};  ///< Number of configured thrusters.
    bool ready_{false};  ///< Whether the allocation matrices are available.

    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;  ///< Maps thruster forces to a body wrench.
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;  ///< Maps a body wrench to thruster forces.
};
