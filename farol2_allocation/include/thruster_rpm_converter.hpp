#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "farol2_interfaces/msg/thruster_rpm.hpp"

class ThrusterRpmConverter {
  public:
    enum class Mode {
      STATIC_CURVE,
      THRUSTER_RUDDER
    };

    /**
     * @brief Create a converter based on calibrated forward and reverse curves.
     */
    static ThrusterRpmConverter staticCurve(
      std::vector<double> coef_fwd,
      std::vector<double> coef_bwd,
      double max_rpm,
      double min_rpm);

    /**
     * @brief Create a converter for the combined thruster-rudder model.
     */
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

    /**
     * @brief Update the surge velocity used by the dynamic propeller model.
     */
    void setSurge(double surge);

    /**
     * @brief Convert per-thruster forces into a timestamped RPM command.
     */
    farol2_interfaces::msg::ThrusterRPM convert(
      const std::vector<double> & forces,
      const rclcpp::Time & stamp) const;

  private:
    /**
     * @brief Create an unconfigured converter for use by the factory methods.
     */
    ThrusterRpmConverter() = default;

    /**
     * @brief Invert the calibrated static force curve and clamp the RPM.
     */
    double forceToRpmStatic(double force) const;

    /**
     * @brief Convert force with the selected thruster-rudder RPM model.
     */
    double forceToRpmThrusterRudder(double force) const;

    Mode mode_{Mode::STATIC_CURVE};  ///< Active force-to-RPM conversion model.
    int thruster_rudder_mode_{1};  ///< Thruster-rudder model variant.
    std::vector<double> coef_fwd_;  ///< Forward static-curve coefficients.
    std::vector<double> coef_bwd_;  ///< Reverse static-curve coefficients.
    double max_rpm_{0.0};  ///< Maximum permitted RPM.
    double min_rpm_{0.0};  ///< Minimum permitted RPM.
    double rho_{0.0};  ///< Fluid density used by the propeller model.
    double k_t_bp_{0.0};  ///< Bollard-pull thrust coefficient.
    double prop_pitch_{0.0};  ///< Propeller pitch.
    double diameter_{0.0};  ///< Propeller diameter.
    double surge_{0.0};  ///< Current surge velocity through the water.
};
