#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <variant>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

Eigen::Matrix<double, 6, Eigen::Dynamic> getThrustAllocationMatrix(
  const std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration,
  const int nr_thrusters);
Eigen::Matrix3d getRotationMatrixThruster2Body(double roll, double pitch, double yaw);
std::variant<std::string, std::vector<double>> getFieldFromParameter(rclcpp::Parameter param, std::string key_name);
std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> getThrusterConfiguration(rclcpp::Node &node);

/**
 * @brief Build thrust allocation matrix based on thruster configuration.
 */
Eigen::Matrix<double, 6, Eigen::Dynamic> getThrustAllocationMatrix(
      std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration,
      int nr_thrusters) {
  /* Create thrust allocation matrix */
  Eigen::Matrix<double, 6, Eigen::Dynamic> thrust_allocation_matrix;
  thrust_allocation_matrix.resize(6, nr_thrusters);
  
  /* Normalised vector of forces in the thruster's referential */
  const Eigen::Vector3d unit_vector(1.0, 0.0, 0.0);

  /* Build matrix column by column */
  for (int i = 0; i < nr_thrusters; i++) {
    /* Compute vector of forces in the body's referential frame */
    Eigen::Vector3d f = getRotationMatrixThruster2Body(std::get<std::vector<double>>(thruster_configuration[i]["angles"])[0]/180*M_PI,
                                                       std::get<std::vector<double>>(thruster_configuration[i]["angles"])[1]/180*M_PI,
                                                       std::get<std::vector<double>>(thruster_configuration[i]["angles"])[2]/180*M_PI)*unit_vector;

    /* Get vector of moment arms */
    Eigen::Vector3d l;
    l << std::get<std::vector<double>>(thruster_configuration[i]["moment_arms"])[0],
         std::get<std::vector<double>>(thruster_configuration[i]["moment_arms"])[1],
         std::get<std::vector<double>>(thruster_configuration[i]["moment_arms"])[2];
    
    /* Each column of the Thrust Allocation Matrix:                            */
    /* tau_i = |    f_i    | <- forces vector                                  */
    /*         | l_i x f_i | <- cross product of moment arms and forces vector */
    thrust_allocation_matrix.block<3,1>(0,i) = f;
    thrust_allocation_matrix.block<3,1>(3,i) = l.cross(f);
  }

  return thrust_allocation_matrix;
}

/**
 * @brief Get rotation matrix from thruster frame to body frame, given the roll,
 * pitch and yaw angles, which are the angles of rotation from body to thruster,
 * in the following order: yaw first, then pitch, finally roll.
 */
Eigen::Matrix3d getRotationMatrixThruster2Body(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).matrix();
}

/**
 * @brief Get field from ROS parameter according to key name.
 */
std::variant<std::string, std::vector<double>> getFieldFromParameter(rclcpp::Node &node, const rclcpp::Parameter &param, const std::string &key_name) {
  /* According to key name, parse ros parameter properly */
  if (key_name == "name") {
    return param.as_string();
  } else if (key_name == "moment_arms" || key_name == "angles") {
    return param.as_double_array();
  } else {
    RCLCPP_WARN(node.get_logger(), "Unknown loaded data from config in thruster configuration.");
    return "Unknown";
  }
}

std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> getThrusterConfiguration(rclcpp::Node &node) {
  std::map<std::string, rclcpp::Parameter> raw_thruster_configuration;
  std::vector<std::map<std::string, std::variant<std::string, std::vector<double>>>> thruster_configuration;
  int idx = 0;
  std::string key_name;

  /* Get raw flatten thruster configuration parameters */
  if (node.get_node_parameters_interface()->get_parameters_by_prefix(
        "actuation.thrusters.configuration", raw_thruster_configuration)) {
    /* Iterate through std::map to create new map with thruster configurations */
    for (const auto & [key, param] : raw_thruster_configuration) {
      /* Get number at the beginning of the key and param name */
      try {
        idx = std::stoi(key.substr(0, key.find('.')));
        key_name = key.substr(key.find('.') + 1);
      } catch (...) {
        continue;
      }
      
      /* Create element in thruster_configuration vector with map or add info to already existing map */
      if (idx >= (int)thruster_configuration.size()) {
        RCLCPP_DEBUG(node.get_logger(), "NEW ENTRY: %d -- %ld", idx, thruster_configuration.size());
        thruster_configuration.push_back({{key_name, getFieldFromParameter(node, param, key_name)}});
      } else {
        thruster_configuration[idx].insert({key_name, getFieldFromParameter(node, param, key_name)});
      }
    }
  }

  return thruster_configuration;
}