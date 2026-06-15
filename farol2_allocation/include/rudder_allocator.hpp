#pragma once

#include <cmath>

#include <Eigen/Dense>

#include <farol2_interfaces/msg/navigation_state.hpp>

struct RudderAllocationResult {
  double rudder_angle_rad{0.0};
  double rudder_x_body_drag{0.0};
  double flow_to_rudder_angle{0.0};
  double gamma{0.0};
};

class RudderAllocator {
  public:
    RudderAllocator(
      double rudder_angle_min_rad,
      double rudder_angle_max_rad,
      double rudder_cm_distance,
      double k_s,
      double k_l,
      double k_d0,
      double k_d1);

    RudderAllocationResult compute(
      const farol2_interfaces::msg::NavigationState & nav_state,
      double tau_r) const;

  private:
    double solveDeltaFromTau(double tau_r, double gamma, double v_sq) const;

    double rudder_angle_min_{0.0};
    double rudder_angle_max_{0.0};
    double rudder_cm_distance_{0.0};
    double k_s_{0.0};
    double k_l_{0.0};
    double k_d0_{0.0};
    double k_d1_{0.0};
};