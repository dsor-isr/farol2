#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace farol_utils {

class SecondOrderLowPass {
public:
  SecondOrderLowPass();

  // Configure with cutoff in rad/s, dampening and time step (assumed constant)
  void configure(double wc, double Ts, std::string design="butterworth", std::string method="tustin", bool wrap_angle=false);

  // Reset filter state (optionally to known initial conditions)
  void reset(double u_hat0 = 0.0, double du_hat0 = 0.0);

  // ZOH step: assumes u is held constant during the sample
  void step(double u);

  // Outputs
  double y()  const;
  double dy() const;
  double ddy() const; 

  bool isConfigured() const { return configured_; }

private:
  void discretize_euler();
  void discretize_zoh();
  void discretize_tustin();

  void design_butterworth();
  void design_bessel();

  bool configured_{false};
  bool wrap_angle_{false};

  double wc_{0.0};
  double Ts_{0.0};
  double last_u_{0.0};

  Eigen::Matrix2d A_{Eigen::Matrix2d::Identity()};
  Eigen::Vector2d B_{Eigen::Vector2d::Zero()};

  Eigen::Matrix2d Ad_{Eigen::Matrix2d::Identity()};
  Eigen::Vector2d Bd_{Eigen::Vector2d::Zero()};

  Eigen::Vector2d x_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_x_{Eigen::Vector2d::Zero()};
};

} // namespace farol_utils
