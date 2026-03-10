#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <complex>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace farol_utils {

class LowPassFilter {
public:
  LowPassFilter();

  // Configure with cutoff in rad/s, dampening and time step (assumed constant)
  void configure(double wc, double Ts, int order=2, std::string design="butterworth", std::string method="tustin", bool wrap_angle=false);

  // Reset filter state (optionally to known initial conditions)
  void reset(double u_hat0 = 0.0, double du_hat0 = 0.0);

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
  std::string method_;

  double wc_{0.0};
  double Ts_{0.0};
  double last_u_{0.0};
  double u_unwrapped_{0.0};
  bool have_u_unwrapped_{false};

  Eigen::MatrixXd A_;
  Eigen::VectorXd B_;

  Eigen::MatrixXd Ad_;
  Eigen::VectorXd Bd_;

  Eigen::VectorXd x_;
  Eigen::VectorXd last_x_;
};

} // namespace farol_utils
