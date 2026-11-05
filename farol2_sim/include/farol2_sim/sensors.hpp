#pragma once

#include <memory>
#include <cstdint>
#include <random>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <farol2_interfaces/msg/depth.hpp>
#include <farol2_interfaces/msg/thruster_rpm.hpp>
#include <farol2_interfaces/msg/control_surface_angle.hpp>
#include "farol2_sim/state.hpp"
#include "farol2_sim/sensor_kinematics.hpp"
#include <tf2_ros/buffer.h>

namespace farol2_sim {

// Read-only inputs shared by all sensors for one completed simulation step.
// References are valid only during publish(); sensors must not retain them.
struct SensorState {
  const State & state;
  const Vector6d & acceleration;  // Body-coordinate derivatives [v_water_dot, omega_dot].
  const Environment & environment;
  rclcpp::Time stamp;
  // Actual post-step actuator states, in configured actuator order.
  std::vector<double> thruster_rpm;
  std::vector<double> control_surface_angles;  // radians internally.
};

class Sensor {
public:
  virtual ~Sensor() = default;
  virtual void initializeGeometry(tf2_ros::Buffer &, const std::string &) {}
  virtual void publish(const SensorState & sample) = 0;
};

// Fixed sensor mounts are looked up once before simulation starts. The node
// retries missing TFs; mounted sensors never silently fall back to identity.
class MountedSensor : public Sensor {
public:
  MountedSensor(rclcpp::Node &, const std::string & name, const std::string & default_frame);
  void initializeGeometry(tf2_ros::Buffer &, const std::string & base_frame) override;
protected:
  SensorMount mount_;
  std::string frame_;
private:
  bool geometry_ready_{false};
};

struct NoiseParameters {
  Eigen::Vector3d bias;
  Eigen::Vector3d standard_deviation;
};

// Each sensor owns its sampler and its sensor.<name> noise configuration.
// noise_enabled controls both bias and random noise for that sensor.
class SensorNoise {
public:
  SensorNoise(rclcpp::Node & node, const std::string & sensor, std::uint32_t seed);
  NoiseParameters load(rclcpp::Node & node, const std::string & name);
  double sample(const NoiseParameters & parameters, int axis);
  bool enabled() const { return enabled_; }
private:
  std::string prefix_;
  bool enabled_;
  std::mt19937 random_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

class GnssSensor final : public MountedSensor {
public:
  explicit GnssSensor(rclcpp::Node & node);
  void publish(const SensorState & sample) override;
private:
  SensorNoise noise_;
  NoiseParameters position_noise_, velocity_noise_;
  int utm_zone_;
  bool northp_;
  double northing_, easting_, origin_altitude_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr utm_pub_, velocity_pub_;
};

class ImuSensor final : public MountedSensor {
public:
  explicit ImuSensor(rclcpp::Node & node);
  void publish(const SensorState & sample) override;
private:
  SensorNoise noise_;
  NoiseParameters orientation_noise_, rate_noise_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

class DvlSensor final : public MountedSensor {
public:
  explicit DvlSensor(rclcpp::Node & node);
  void publish(const SensorState & sample) override;
private:
  SensorNoise noise_;
  NoiseParameters velocity_noise_;
  bool bottom_track_;
  std::string output_frame_, base_frame_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;
};

class DepthCellSensor final : public MountedSensor {
public:
  explicit DepthCellSensor(rclcpp::Node & node);
  void publish(const SensorState & sample) override;
private:
  SensorNoise noise_;
  NoiseParameters position_noise_;
  rclcpp::Publisher<farol2_interfaces::msg::Depth>::SharedPtr publisher_;
};

// Synthetic test signal only: no seabed geometry or vehicle-position dependence.
class AltimeterSensor final : public MountedSensor {
public:
  explicit AltimeterSensor(rclcpp::Node & node);
  void publish(const SensorState & sample) override;
private:
  double period_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

class ThrusterRpmSensor final : public Sensor {
public:
  ThrusterRpmSensor(rclcpp::Node & node, const std::string & base_frame);
  void publish(const SensorState & sample) override;
private:
  std::string frame_;
  rclcpp::Publisher<farol2_interfaces::msg::ThrusterRPM>::SharedPtr publisher_;
};

class ControlSurfaceAngleSensor final : public Sensor {
public:
  ControlSurfaceAngleSensor(rclcpp::Node & node, const std::string & base_frame);
  void publish(const SensorState & sample) override;
private:
  std::string frame_;
  rclcpp::Publisher<farol2_interfaces::msg::ControlSurfaceAngle>::SharedPtr publisher_;
};

}  // namespace farol2_sim
