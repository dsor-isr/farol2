#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace rclcpp {
class Node;
class Parameter;
}

namespace farol2_utils {

// Decode an existing double-array parameter. These functions validate type,
// size and finite values, without declaring or changing parameters.
std::vector<double> vectorParameter(const rclcpp::Parameter &, std::size_t size);
Eigen::Vector3d vector3Parameter(const rclcpp::Parameter &);
// Accepts `size` diagonal entries or `size * size` row-major entries.
Eigen::MatrixXd matrixParameter(const rclcpp::Parameter &, int size);

// Declare required parameters. A second declaration intentionally raises the
// usual ROS ParameterAlreadyDeclaredException; use get* for existing parameters.
std::vector<double> declareVectorParameter(rclcpp::Node &, const std::string &, std::size_t size);
Eigen::Vector3d declareVector3Parameter(rclcpp::Node &, const std::string &);
Eigen::MatrixXd declareMatrixParameter(rclcpp::Node &, const std::string &, int size);

// Read already-declared parameters, including automatically declared overrides.
std::vector<double> getVectorParameter(rclcpp::Node &, const std::string &, std::size_t size);
Eigen::Vector3d getVector3Parameter(rclcpp::Node &, const std::string &);
Eigen::MatrixXd getMatrixParameter(rclcpp::Node &, const std::string &, int size);

}  // namespace farol2_utils
