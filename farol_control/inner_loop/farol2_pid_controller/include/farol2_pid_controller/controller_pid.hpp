#ifndef CONTROLLER_PID_HPP_
#define CONTROLLER_PID_HPP_

#include <string>
#include <farol2_utils/filters/low_pass_filter.hpp>

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
                 double tau_min, double tau_max, bool use_lpf, double lpf_wc,
                 int lpf_order, std::string lpf_design, std::string lpf_method, bool wrapToPi);

  double callController(double state, double state_ref, double state_rate, double dt);

  void setGains(double kp, double ki, double kd, double kffv_lin, double kffv_sq, double kffa);

  double getError() { return error_; }
  double getIntegralTerm() { return i_term_; }
  double getProportionalTerm() { return p_term_; }
  double getDerivativeTerm() { return d_term_; }
  double getTau_d() { return tau_d_; }
  double getTau_sat() { return tau_sat_; }
  double getAntiWindupTerm() { return Ka_ * (tau_prev_ - tau_sat_prev_); }
  double getTauDot() { return tau_dot_; }
  double getTau() { return tau_; }

  double ref_raw_;
  double state_;
  double ref_;
  double dref_;
  double ddref_;
  double dddref_;
  double kffv_lin_;
  double kffv_sq_;
  double kffa_;
  double p_term_;
  double i_term_;
  double d_term_;
  double error_dot_;
  double error_rate_dot_;

  double kp_;
  double ki_;
  double kd_;
  double lpf_wc_;
  double tau_min_;
  double tau_max_;
  bool wrapToPi_;
  bool use_lpf_;
  bool configured_ = false;

  double error_ = 0.0;
  double error_rate_ = 0.0;
  double tau_d_ = 0.0;

  bool first_it_ = true;
  double error_dot_internal_ = 0.0;
  double error_rate_dot_internal_ = 0.0;
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

  double state_rate_dot_filter_ = 0.0;
  double state_rate_dot_filter_prev_ = 0.0;
  double error_rate_dot_filter_ = 0.0;
  double error_rate_dot_filter_prev_ = 0.0;
  double lpf_A_ = 0.0;
  double lpf_B_ = 0.0;

  farol2_utils::LowPassFilter lpf_;

 private:
};

}  // namespace farol_control

#endif  // CONTROLLER_PID_HPP_