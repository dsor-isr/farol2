#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <complex>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace farol2_utils {

class LowPassFilter {
public:
  LowPassFilter();

  // Legacy signature kept for backward compatibility.
  // Configure with cutoff in rad/s and nominal/fixed time step.
  // If use_fixed_Ts is false, runtime dt passed to step() is used.
  // If use_fixed_Ts is true, Ts is always used and step dt is ignored.
  void configure(double wc, double Ts, int order=2, std::string design="butterworth", std::string method="tustin", bool wrap_angle=false, bool use_fixed_Ts=false);

  // Overload with optional trailing Ts for fixed-step mode.
  // Use this when variable-step mode is desired and Ts should be omitted.
  void configure(double wc, int order=2, std::string design="butterworth", std::string method="tustin", bool wrap_angle=false, bool use_fixed_Ts=false, double Ts=0.0);

  // Reset filter state (optionally to known initial conditions)
  void reset(double u_hat0 = 0.0, double du_hat0 = 0.0);

  // Step using configured fixed Ts (requires use_fixed_Ts=true).
  void step(double u);

  void step(double u, double dt);

  // Outputs
  double y()  const;
  double dy() const;
  double ddy() const; 

  bool isConfigured() const { return configured_; }

private:
  void discretize_euler(double dt);
  void discretize_zoh(double dt);
  void discretize_tustin(double dt);

  void design_butterworth();
  void design_bessel();

  int n_{2};

  bool configured_{false};
  bool wrap_angle_{false};
  bool use_fixed_Ts_{false};
  std::string method_;

  double wc_{0.0};
  double Ts_{0.0};
  double last_u_{0.0};
  double last_dy_{0.0};  // For computing ddy in order 1 and when order < 3
  double last_dt_{0.0};  // Track actual dt for ddy calculation
  double u_unwrapped_{0.0};
  bool have_u_unwrapped_{false};

  Eigen::MatrixXd A_;
  Eigen::VectorXd B_;

  Eigen::MatrixXd Ad_;
  Eigen::VectorXd Bd_;

  Eigen::VectorXd x_;
  Eigen::VectorXd last_x_;
};

} // namespace farol2_utils
