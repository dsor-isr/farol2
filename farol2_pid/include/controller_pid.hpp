#ifndef CONTROLLER_PID_HPP_
#define CONTROLLER_PID_HPP_

#include <string>

/**
 * @file controller_pid.hpp
 * @brief PID (Proportional-Integral-Derivative) controller with feed-forward and filtering
 * @author Ravi Regalo
 */

namespace farol_control {

class ControllerPID {
 public:
  ControllerPID() = default;

  void configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq, double kffa,
   double tau_min, double tau_max, bool delta_implementation, bool wrapToPi);

  // Backward-compatible overload: defaults to delta implementation enabled.
    void configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq, double kffa,
      double tau_min, double tau_max, bool wrapToPi);

  double callController(double state, double state_ref, double state_rate,
        double dref, double ddref, double dt);

  void setGains(double kp, double ki, double kd, double kffv_lin, double kffv_sq, double kffa);

  double getError() { return error_; }
  double getIntegralTerm() { return i_term_; }
  double getProportionalTerm() { return p_term_; }
  double getDerivativeTerm() { return d_term_; }
  double getFFTerm() { return ff_term_; }
  double getTau_d() { return tau_d_; }
  double getTau_sat() { return tau_sat_; }
  double getAntiWindupTerm() { return Ka_ * (tau_prev_ - tau_sat_prev_); }
  double getTauDot() { return tau_dot_; }
  double getTau() { return tau_; }
  double getOutput() { return output_; }

  double state_;
  double ref_;
  double dref_;
  double ddref_;
  double kffv_lin_;
  double kffv_sq_;
  double kffa_;
  double p_term_{0.0};
  double i_term_{0.0};
  double d_term_{0.0};
  double ff_term_{0.0};
  double error_dot_{0.0};
  double error_rate_dot_{0.0};
  double state_rate_raw_ = 0.0;
  double state_rate_used_ = 0.0;
  double state_used_for_control_ = 0.0;
  double ref_used_for_control_ = 0.0;

  double kp_;
  double ki_;
  double kd_;
  double tau_min_;
  double tau_max_;
  bool wrapToPi_;
  bool configured_ = false;
  bool delta_implementation_ = true;

  double error_ = 0.0;
  double error_rate_ = 0.0;
  double tau_d_ = 0.0;

  bool first_it_ = true;
  double state_rate_dot_ = 0.0;
  double ddref_dot_ = 0.0;
  double state_dot_ = 0.0;
  double state_prev_ = 0.0;
  double error_prev_ = 0.0;
  double error_rate_prev_ = 0.0;
  double state_rate_prev_ = 0.0;
  double ddref_prev_ = 0.0;

  double Ka_ = 0.0;
  double tau_dot_ = 0.0;
  double tau_ = 0.0;
  double tau_prev_ = 0.0;
  double tau_sat_ = 0.0;
  double tau_sat_prev_ = 0.0;
  double output_ = 0.0;

 private:
};

}  // namespace farol_control

#endif  // CONTROLLER_PID_HPP_