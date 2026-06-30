#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"

struct ThrusterGeometry {
  Eigen::Vector3d force_axis_body{1.0, 0.0, 0.0};
  Eigen::Vector3d moment_arm_body{0.0, 0.0, 0.0};
};

inline ThrusterGeometry thrusterGeometryFromTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q_msg = transform.transform.rotation;

  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);

  ThrusterGeometry geometry;
  geometry.moment_arm_body << t.x, t.y, t.z;
  geometry.force_axis_body = q.normalized().toRotationMatrix() * Eigen::Vector3d::UnitX();
  return geometry;
}

inline bool buildThrusterGeometryFromTF(
  tf2_ros::Buffer & tf_buffer,
  const rclcpp::Clock & clock,
  const rclcpp::Logger & logger,
  const std::string & base_frame,
  const std::vector<std::string> & thruster_frames,
  std::vector<ThrusterGeometry> & geometry)
{
  geometry.clear();
  geometry.reserve(thruster_frames.size());

  for (const auto & thruster_frame : thruster_frames) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer.lookupTransform(
        base_frame,
        thruster_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger,
        clock,
        5000,
        "Waiting for transform %s -> %s: %s",
        base_frame.c_str(),
        thruster_frame.c_str(),
        ex.what());
      return false;
    }

    geometry.push_back(thrusterGeometryFromTransform(transform));
  }

  return true;
}

inline Eigen::Matrix<double, 6, Eigen::Dynamic> buildThrustAllocationMatrix(
  const std::vector<ThrusterGeometry> & geometry)
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> matrix;
  matrix.resize(6, geometry.size());

  for (size_t i = 0; i < geometry.size(); ++i) {
    matrix.block<3, 1>(0, i) = geometry[i].force_axis_body;
    matrix.block<3, 1>(3, i) =
      geometry[i].moment_arm_body.cross(geometry[i].force_axis_body);
  }

  return matrix;
}

inline Eigen::MatrixXd buildSimulatorThrusterMatrix(
  const std::vector<ThrusterGeometry> & geometry)
{
  Eigen::MatrixXd matrix(geometry.size(), 6);

  for (size_t i = 0; i < geometry.size(); ++i) {
    matrix.block<1, 3>(i, 0) = geometry[i].force_axis_body.transpose();
    matrix.block<1, 3>(i, 3) = geometry[i].moment_arm_body.transpose();
  }

  return matrix;
}
