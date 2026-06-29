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
    StaticThrusterAllocator(
      std::string base_frame,
      std::vector<std::string> thruster_frames);

    bool initialize(tf2_ros::Buffer & tf_buffer, const rclcpp::Clock & clock, const rclcpp::Logger & logger);
    bool ready() const;
    size_t thrusterCount() const;
    Eigen::VectorXd allocate(const Eigen::Vector<double, 6> & tau) const;

  private:
    bool buildAllocationMatrix(
      tf2_ros::Buffer & tf_buffer,
      const rclcpp::Clock & clock,
      const rclcpp::Logger & logger);

    std::string base_frame_;
    std::vector<std::string> thruster_frames_;
    size_t nr_thrusters_{0};
    bool ready_{false};

    Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix_;
    Eigen::Matrix<double, Eigen::Dynamic, 6> thrust_allocation_matrix_pseudo_inv_;
};
