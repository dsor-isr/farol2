#include "farol2_utils/parameter_utils.hpp"
#include "farol2_utils/validation.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

namespace farol2_utils {
namespace {
std::vector<double> doubleArray(const rclcpp::Parameter & parameter) {
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    throw std::invalid_argument(parameter.get_name() + " must be a double array");
  }
  const auto values = parameter.as_double_array();
  requireFiniteValues(values, parameter.get_name());
  return values;
}

rclcpp::Parameter declareArray(rclcpp::Node & node, const std::string & name) {
  return rclcpp::Parameter(name, node.declare_parameter<std::vector<double>>(name));
}
}  // namespace

std::vector<double> vectorParameter(const rclcpp::Parameter & parameter, std::size_t size) {
  auto values = doubleArray(parameter);
  requireSize(values, size, parameter.get_name());
  return values;
}

Eigen::Vector3d vector3Parameter(const rclcpp::Parameter & parameter) {
  const auto values = vectorParameter(parameter, 3);
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::MatrixXd matrixParameter(const rclcpp::Parameter & parameter, int size) {
  requirePositive(size, parameter.get_name() + " matrix dimension");
  const auto values = doubleArray(parameter);
  const auto dimension = static_cast<std::size_t>(size);
  if (values.size() != dimension && values.size() != dimension * dimension) {
    throw std::invalid_argument(parameter.get_name() + " must be a diagonal or a row-major square matrix");
  }
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      matrix(i, j) = values.size() == dimension ?
        (i == j ? values[i] : 0.0) : values[static_cast<std::size_t>(i) * dimension + j];
    }
  }
  return matrix;
}

std::vector<double> declareVectorParameter(rclcpp::Node & node, const std::string & name, std::size_t size) {
  return vectorParameter(declareArray(node, name), size);
}

Eigen::Vector3d declareVector3Parameter(rclcpp::Node & node, const std::string & name) {
  return vector3Parameter(declareArray(node, name));
}

Eigen::MatrixXd declareMatrixParameter(rclcpp::Node & node, const std::string & name, int size) {
  return matrixParameter(declareArray(node, name), size);
}

std::vector<double> getVectorParameter(rclcpp::Node & node, const std::string & name, std::size_t size) {
  return vectorParameter(node.get_parameter(name), size);
}

Eigen::Vector3d getVector3Parameter(rclcpp::Node & node, const std::string & name) {
  return vector3Parameter(node.get_parameter(name));
}

Eigen::MatrixXd getMatrixParameter(rclcpp::Node & node, const std::string & name, int size) {
  return matrixParameter(node.get_parameter(name), size);
}
}  // namespace farol2_utils
