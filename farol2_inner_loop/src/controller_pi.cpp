#include "farol2_inner_loop/controller_pi.hpp"

#include <algorithm>
#include <cmath>

namespace farol_control {

void ControllerPI::configure(double kp, double ki, double tau_min, double tau_max, bool use_lpf, double lpf_wc) {
  /* Set parameters */
  kp_ = kp;
  ki_ = ki;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
  use_lpf_ = use_lpf;
  lpf_wc_ = lpf_wc;
  configured_ = true;
}

double ControllerPI::callController(double state, double state_ref, double dt) {
  if (!configured_) return 0.0;
  /* Compute error */
  error_ = state_ref - state;

  /* Compute derivative of Kp term if not in first iteration */
  if (!first_it_) {
    state_dot_ = (state - state_prev_) / dt;
    if (use_lpf_) {
      /* Apply low pass filter due to noise amplification from derivative computation */
      lpf_A_ = std::exp(-lpf_wc_ * dt);
      lpf_B_ = 1 - lpf_A_;
      state_dot_filter_ = lpf_A_ * state_dot_filter_prev_ + lpf_B_ * state_dot_;
    } else {
      state_dot_filter_ = state_dot_;
    }
  } else {
    /* Reset first iteration flag */
    first_it_ = false;
  }

  /* Add all PI terms */
  tau_d_ = ki_ * error_ - kp_ * state_dot_filter_;

  /* Anti-windup */
  Ka_ = 1.0 / dt;
  tau_dot_ = tau_d_ - Ka_ * (tau_prev_ - tau_sat_prev_);
  tau_ = tau_prev_ + tau_dot_ * dt;
  tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);

  state_prev_ = state;
  state_dot_filter_prev_ = state_dot_filter_;
  tau_prev_ = tau_;
  tau_sat_prev_ = tau_sat_;

  return tau_sat_;
}

void ControllerPI::setGains(double kp, double ki) {
  kp_ = kp;
  ki_ = ki;
}

}  // namespace farol_control
