#pragma once

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "farol2_sim/actuators.hpp"
#include "farol2_sim/integrator.hpp"
#include "farol2_sim/kinematics.hpp"
#include "farol2_sim/sensors.hpp"
#include "farol2_interfaces/msg/thruster_rpm.hpp"
#include "farol2_interfaces/msg/control_surface_angle.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

class VehicleSimNode : public rclcpp::Node {
public:
  VehicleSimNode();

private:
  void loadParameters();
  void initialize_subscribers();
  void initialize_publishers();
  void initialize_sensors();
  void initializeGeometry();
  void tick();
  void publishWorldTransform(const rclcpp::Time & stamp);

  farol2_sim::State state_;
  farol2_sim::Environment environment_;
  farol2_sim::Vector6d acceleration_{farol2_sim::Vector6d::Zero()};
  std::unique_ptr<farol2_sim::Dynamics> dynamics_;
  std::vector<farol2_sim::Thruster> thrusters_;
  std::vector<farol2_sim::ActuatorGeometry> thruster_geometry_;
  std::unique_ptr<farol2_sim::Rudder> rudder_;
  farol2_sim::ActuatorGeometry rudder_geometry_;
  std::vector<double> rpm_commands_;
  double rudder_command_{0.0}, rudder_rate_command_{0.0};
  bool rudder_rate_mode_{false}, fixed_rpm_{false};
  double fixed_rpm_value_{0.0}, rudder_speed_{0.0}, rudder_min_{0.0}, rudder_max_{0.0};
  std::string rudder_joint_, base_frame_, frame_prefix_;
  std::vector<std::string> thruster_frames_;
  bool geometry_ready_{false};
  double node_period_{0.01};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_, geometry_timer_;
  rclcpp::Subscription<farol2_interfaces::msg::ThrusterRPM>::SharedPtr rpm_sub_;
  rclcpp::Subscription<farol2_interfaces::msg::ControlSurfaceAngle>::SharedPtr rudder_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_rate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_, body_velocity_pub_,
    orientation_pub_, orientation_rate_pub_, body_acceleration_pub_, angular_acceleration_pub_;
  std::vector<std::unique_ptr<farol2_sim::Sensor>> sensors_;
};
