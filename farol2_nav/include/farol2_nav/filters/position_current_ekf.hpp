#pragma once

#include <farol2_nav/filters/base_filter.hpp>
#include <farol2_nav/srv/tune_position_ekf.hpp>

#include <Eigen/Dense>

namespace farol2_nav
{
namespace filters
{

class PositionCurrentEkfFilter : public BaseFilter
{
public:
  std::string name() const override { return "position_current_ekf"; }
  void configure(rclcpp::Node & node) override;
  void compute(double dt_s, const MeasurementSnapshot & measurements, State & state) override;

private:
  double rpm_to_body_speed_mps(const MeasurementSnapshot & m, double dt_s);
  void on_tune_ekf(
    const std::shared_ptr<farol2_nav::srv::TunePositionEkf::Request> req,
    std::shared_ptr<farol2_nav::srv::TunePositionEkf::Response> res);

  bool initialized_{false};
  Eigen::Vector4d x_{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d P_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d Q_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix2d R_{Eigen::Matrix2d::Identity()};
  Eigen::Matrix<double, 2, 4> H_{Eigen::Matrix<double, 2, 4>::Zero()};

  // Position EKF parameters.
  double q_pos_{0.1};
  double q_current_{0.01};
  double r_pos_{1.0};
  double p0_pos_{25.0};
  double p0_current_{1.0};
  bool override_position_state_{true};

  // Surge dynamics model parameters.
  double rpm_min_{-2000.0};
  double rpm_max_{2000.0};
  double rpm_rate_limit_{200.0};
  double rho_{1025.0};
  double prop_pitch_{0.381};
  double prop_diameter_{0.4318};
  double k_t_bp_{0.061461915};
  double m_u_{3100.0};
  double x_u_{0.0};
  double x_uu_{-96.2270};

  // Optional surge override parameters.
  double override_velocity_{0.0};
  double override_rpms_{600.0};
  double override_timeout_s_{5.0};

  // Runtime model state.
  bool rpm_model_initialized_{false};
  double rpm_model_state_{0.0};
  double u_estimated_{0.0};
  double time_in_override_zone_s_{0.0};

  rclcpp::Service<farol2_nav::srv::TunePositionEkf>::SharedPtr tune_ekf_srv_{};
};

}  // namespace filters
}  // namespace farol2_nav
