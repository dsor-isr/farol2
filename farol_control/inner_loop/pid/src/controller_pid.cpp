#include "controller_pid.hpp"

#include <algorithm>
#include <cmath>
#include <farol_utils/angles.hpp>

namespace farol_control {

/* Constructor */
ControllerPID::ControllerPID(double kp, double ki, double kd, double lpf_wc, double tau_min,
                             double tau_max, double kffv_lin, double kffv_sq, double kffa,
                             bool wrapToPi, int lpf_order, std::string lpf_method,
                             std::string lpf_design) {
  /* Set parameters */
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
  lpf_wc_ = (lpf_wc > 0.0) ? lpf_wc : 1.0;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
  wrapToPi_ = wrapToPi;

  lpf_.configure(lpf_wc_, 0.1, lpf_order, lpf_design, lpf_method, wrapToPi_);
}

// Delta implementation for PID
double ControllerPID::callController(double state, double state_ref, double state_rate, double dt) {
  ref_raw_ = state_ref;
  state_ = state;

  // Pass reference signal through LPF to extract reference derivatives for ff terms
  lpf_.step(state_ref, dt);
  ref_ = lpf_.y();
  dref_ = lpf_.dy();
  ddref_ = lpf_.ddy();

  // Compute error
  error_ = state - state_ref;
  if (wrapToPi_)  // Wrap to [-pi, pi] if needed
    error_ = farol_utils::wrapToPi(error_);
  // Compute error derivative
  error_rate_ = state_rate - dref_;

  // this is outside so we dont miss the initial step so that delta behaves more like tradition pid
  error_dot_ = (error_ - error_prev_) / dt;
  // Compute derivative of all terms except the integral
  if (!first_it_) {
    state_rate_dot_ = (state_rate - state_rate_prev_) / dt;
    state_dot_ = farol_utils::wrapToPi(state_ - state_prev_) / dt;
    error_rate_dot_ = (error_rate_ - error_rate_prev_) / dt;
    ddref_dot_ = (ddref_ - ddref_prev_) / dt;
  } else
    first_it_ = false;  // Reset first iteration flag

  /* Add all PID terms */
  tau_d_ = -ki_ * error_ - kp_ * error_dot_ - kd_ * error_rate_dot_ + kffa_ * ddref_dot_ +
           kffv_lin_ * state_rate_dot_ + kffv_sq_ * state_rate_dot_ * abs(state_rate_dot_);

  // for debug only
  p_term_ = -kp_ * error_ * dt;
  i_term_ = -ki_ * error_ * dt;
  d_term_ = -kd_ * error_rate_ * dt;

  /* Anti-windup */
  Ka_ = 1.0 / dt;
  tau_dot_ = tau_d_ - Ka_ * (tau_prev_ - tau_sat_prev_);
  tau_ = tau_prev_ + tau_dot_ * dt;
  tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);

  /* Set prev values */
  error_prev_ = error_;
  state_prev_ = state_;
  state_rate_prev_ = state_rate;
  error_rate_prev_ = error_rate_;
  state_rate_dot_filter_prev_ = state_rate_dot_filter_;
  tau_prev_ = tau_;
  tau_sat_prev_ = tau_sat_;
  ddref_prev_ = ddref_;

  return tau_sat_;
}

void ControllerPID::setParams(double kp, double ki, double kd, double lpf_wc, double tau_min,
                              double tau_max, double kffv_lin, double kffv_sq, double kffa) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
  lpf_wc_ = lpf_wc;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
}

}  // namespace farol_control
