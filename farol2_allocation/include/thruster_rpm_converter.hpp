#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "farol2_allocation/msg/thruster_force.hpp"
#include "farol2_interfaces/msg/thruster_rpm.hpp"

class ThrusterRpmConverter {
  public:
    enum class Mode {
      STATIC_CURVE,
      THRUSTER_RUDDER
    };

    static ThrusterRpmConverter staticCurve(
      std::vector<double> coef_fwd,
      std::vector<double> coef_bwd,
      double max_rpm,
      double min_rpm);

    static ThrusterRpmConverter thrusterRudder(
      int mode,
      std::vector<double> coef_fwd,
      std::vector<double> coef_bwd,
      double rho,
      double k_t_bp,
      double prop_pitch,
      double diameter,
      double max_rpm,
      double min_rpm);

    void setSurge(double surge);
    farol2_interfaces::msg::ThrusterRPM convert(
      const farol2_allocation::msg::ThrusterForce & force_msg,
      const rclcpp::Time & stamp) const;

  private:
    ThrusterRpmConverter() = default;

    double forceToRpmStatic(double force) const;
    double forceToRpmThrusterRudder(double force) const;

    Mode mode_{Mode::STATIC_CURVE};
    int thruster_rudder_mode_{1};
    std::vector<double> coef_fwd_;
    std::vector<double> coef_bwd_;
    double max_rpm_{0.0};
    double min_rpm_{0.0};
    double rho_{0.0};
    double k_t_bp_{0.0};
    double prop_pitch_{0.0};
    double diameter_{0.0};
    double surge_{0.0};
};