#pragma once

#include <cmath>

#include <Eigen/Dense>

#include <farol2_interfaces/msg/navigation_state.hpp>

struct RudderAllocationResult {
  double rudder_angle_rad{0.0};  ///< Commanded rudder angle in radians.
  double rudder_x_body_drag{0.0};  ///< Rudder drag along the body surge axis.
  double flow_to_rudder_angle{0.0};  ///< Angle between incoming flow and the rudder.
  double gamma{0.0};  ///< Incoming flow angle relative to the body.
};

class RudderAllocator {
  public:
    /**
     * @brief Create an allocator using the rudder limits and hydrodynamic model.
     */
    RudderAllocator(
      double rudder_angle_min_rad,
      double rudder_angle_max_rad,
      double rudder_cm_distance,
      double k_s,
      double k_l,
      double k_d0,
      double k_d1);

    /**
     * @brief Compute the rudder angle and its surge drag for a requested yaw torque.
     */
    RudderAllocationResult compute(
      const farol2_interfaces::msg::NavigationState & nav_state,
      double tau_r) const;

  private:
    /**
     * @brief Solve the model equation for rudder deflection.
     */
    double solveDeltaFromTau(double tau_r, double gamma, double v_sq) const;

    double rudder_angle_min_{0.0};  ///< Minimum permitted rudder angle.
    double rudder_angle_max_{0.0};  ///< Maximum permitted rudder angle.
    double rudder_cm_distance_{0.0};  ///< Distance from the vehicle centre to the rudder.
    double k_s_{0.0};  ///< Static rudder torque coefficient.
    double k_l_{0.0};  ///< Rudder lift coefficient.
    double k_d0_{0.0};  ///< Constant rudder drag coefficient.
    double k_d1_{0.0};  ///< Quadratic rudder drag coefficient.
};
