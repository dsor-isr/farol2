#pragma once

#include <farol2_nav/filters/base_filter.hpp>
#include <farol2_nav/srv/tune_yaw_rate_ekf.hpp>

#include <farol2_utils/filters/moving_average_filter.hpp>

#include <Eigen/Dense>

#include <deque>

#include <std_msgs/msg/float32.hpp>

namespace farol2_nav
{
namespace filters
{

class YawRateEkfFilter : public BaseFilter
{
public:
  std::string name() const override { return "yaw_rate_ekf"; }
  void configure(rclcpp::Node & node) override;
  void compute(double dt_s, const MeasurementSnapshot & measurements, State & state) override;

private:
  struct HistoryEntry
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d x_post{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d P_post{Eigen::Matrix3d::Identity()};
    double dt_s{0.0};
    double tau_r{0.0};
  };

  double get_torque(
    double rudder_angle_rad,
    const Eigen::Vector3d & velocity_through_water_body,
    double yaw_rate_rad_s) const;
  void predict(Eigen::Vector3d & x, Eigen::Matrix3d & P, double dt_s, double tau_r, const State & s) const;
  void update(Eigen::Vector3d & x, Eigen::Matrix3d & P, double z_r) const;
  void on_tune_ekf(
    const std::shared_ptr<farol2_nav::srv::TuneYawRateEkf::Request> req,
    std::shared_ptr<farol2_nav::srv::TuneYawRateEkf::Response> res);

  bool initialized_{false};
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()};   // [r, b, g]
  Eigen::Matrix3d P_{Eigen::Matrix3d::Identity()};

  // EKF parameters.
  double q_r_{0.02};
  double q_b_{0.01};
  double q_g_{1e-3};
  double r_meas_{0.1};
  double p0_r_{1.0};
  double p0_b_{1.0};
  double p0_g_{1.0};

  // Yaw dynamics parameters.
  double m_r_{88.478840};
  double m_uv_{-2.967651};
  double damping_linear_{14.334052};
  double damping_quadratic_{260.675031};
  double Ks_{1.0};
  double rudder_cm_distance_{4.0};
  double rudder_angle_limit_rad_{0.6283185307179586};
  double K_L_{1.0};
  double K_D0_{0.0};
  double K_D1_{0.0};

  // IMU yaw-rate prefilter.
  std::size_t measurement_window_samples_{5U};
  std::size_t measurement_delay_samples_{2U};
  std::size_t history_max_samples_{300U};
  farol2_utils::MovingAverageFilter yaw_rate_maf_{};

  std::deque<HistoryEntry, Eigen::aligned_allocator<HistoryEntry>> history_{};
  rclcpp::Service<farol2_nav::srv::TuneYawRateEkf>::SharedPtr tune_ekf_srv_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_bias_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_rate_filtered_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_yaw_rate_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr innovation_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr delayed_yaw_rate_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tau_r_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_gain_pub_{};
  std_msgs::msg::Float32 torque_bias_msg_{};
  std_msgs::msg::Float32 torque_gain_msg_{};
  std_msgs::msg::Float32 yaw_rate_filtered_msg_{};
  std_msgs::msg::Float32 current_yaw_rate_msg_{};
  std_msgs::msg::Float32 delayed_yaw_rate_msg_{};
  std_msgs::msg::Float32 tau_r_msg_{};
};

}  // namespace filters
}  // namespace farol2_nav
