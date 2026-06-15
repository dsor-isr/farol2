#pragma once

#include <farol2_nav/filters/base_filter.hpp>

#include <geometry_msgs/msg/vector3.hpp>

namespace farol2_nav
{
namespace filters
{

class AsvDynamicsModelFilter : public BaseFilter
{
public:
  std::string name() const override { return "asv_dynamics_model"; }
  void configure(rclcpp::Node & node) override;
  void compute(double dt_s, const MeasurementSnapshot & measurements, State & state) override;

private:
  double get_tau_r(double rudder_angle_rad, double u, double v, double r) const;

  // Surge/sway model parameters.
  double rpm_min_{-2000.0};
  double rpm_max_{2000.0};
  double rpm_rate_limit_{200.0};
  double rho_{1025.0};
  double prop_pitch_{0.381};
  double prop_diameter_{0.4318};
  double k_t_bp_{0.061461915};
  double m_u_{3100.0};
  double m_uv_{-2.967651};
  double m_v_{3102.967651};
  double x_u_{0.0};
  double x_uu_{-96.2270};
  double Y_v_{-500.0};
  double Y_vv_{-1500.0};

  // Yaw model / rudder-to-torque parameters.
  double m_r_{88.478840};
  double damping_linear_{14.334052};
  double damping_quadratic_{260.675031};
  double rudder_cm_distance_{4.0};
  double rudder_angle_limit_rad_{0.6283185307179586};
  double K_L_{1.398093};
  double K_D0_{0.000575};
  double K_D1_{0.520834};
  double torque_bias_{0.0};

  // Optional initial conditions.
  double init_u_{0.0};
  double init_v_{0.0};
  double init_r_{0.0};

  // Runtime model state.
  bool initialized_{false};
  double rpm_model_state_{0.0};
  double u_model_{0.0};
  double v_model_{0.0};
  double r_model_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr uvr_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tau_pub_{};
  geometry_msgs::msg::Vector3 uvr_msg_{};
  geometry_msgs::msg::Vector3 tau_msg_{};
};

}  // namespace filters
}  // namespace farol2_nav
