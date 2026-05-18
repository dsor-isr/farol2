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
                 double tau_min, double tau_max, bool use_ref_lpf, double lpf_wc,
                 int lpf_order, std::string lpf_design, std::string lpf_method,
                 bool delta_implementation, bool wrapToPi,
                 bool use_state_lpf = false,
                 bool use_state_lpf_for_state_rate = false,
                 bool use_filtered_state_for_control = false,
                 bool use_filtered_ref_for_control = false,
                 bool use_rate_limiter = false,
                 double rate_limit = 0.0);

    // Backward-compatible overload: defaults to delta implementation enabled.
    void configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq, double kffa,
                                 double tau_min, double tau_max, bool use_ref_lpf, double lpf_wc,
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
    double state_raw_;
  double state_;
    double state_filt_;
  double ref_;
  double dref_;
  double ddref_;
  double dddref_;
  double kffv_lin_;
  double kffv_sq_;
  double kffa_;
  double p_term_{0.0};
  double i_term_{0.0};
  double d_term_{0.0};
  double error_dot_{0.0};
  double error_rate_dot_{0.0};
  double state_rate_raw_ = 0.0;
  double state_rate_lpf_ = 0.0;
  double state_rate_used_ = 0.0;
  double state_used_for_control_ = 0.0;
  double ref_used_for_control_ = 0.0;

  double kp_;
  double ki_;
  double kd_;
  double lpf_wc_;
  double tau_min_;
  double tau_max_;
  bool wrapToPi_;
  bool use_ref_lpf_;
    bool use_state_lpf_;
    bool use_state_lpf_for_state_rate_;
    bool use_filtered_state_for_control_;
    bool use_filtered_ref_for_control_;
    bool use_rate_limiter_;
  double rate_limit_ = 0.0;
  double ref_rate_limited_ = 0.0;
  bool rate_limiter_initialized_ = false;
  bool configured_ = false;
  bool delta_implementation_ = true;

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
  double output_ = 0.0;

  double state_rate_dot_filter_ = 0.0;
  double state_rate_dot_filter_prev_ = 0.0;
  double error_rate_dot_filter_ = 0.0;
  double error_rate_dot_filter_prev_ = 0.0;
  double lpf_A_ = 0.0;
  double lpf_B_ = 0.0;

  farol2_utils::LowPassFilter lpf_;
    farol2_utils::LowPassFilter state_lpf_;

 private:
};

}  // namespace farol_control

#endif  // CONTROLLER_PID_HPP_