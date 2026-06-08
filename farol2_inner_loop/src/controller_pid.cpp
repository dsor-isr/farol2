#include "farol2_inner_loop/controller_pid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <farol2_utils/angles.hpp>

namespace farol_control {

void ControllerPID::configure(double kp, double ki, double kd, 
                              double kffv_lin, double kffv_sq, double kffa, 
                              double tau_min, double tau_max,
                              bool delta_implementation, 
                              bool wrapToPi,
                              double state_rate_diff_wc) {
  /* Set parameters */
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
  wrapToPi_ = wrapToPi;
  delta_implementation_ = delta_implementation;
  state_rate_diff_wc_ = std::max(state_rate_diff_wc, 1e-6);
  configured_ = true;
}

// overloads for being able to use less parameters 
void ControllerPID::configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq,
                              double kffa, double tau_min, double tau_max, bool delta_implementation, bool wrapToPi) {
  configure(kp, ki, kd, kffv_lin, kffv_sq, kffa, tau_min, tau_max, delta_implementation, wrapToPi, 1.0);
}
void ControllerPID::configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq,
                              double kffa, double tau_min, double tau_max, bool wrapToPi) {
  configure(kp, ki, kd, kffv_lin, kffv_sq, kffa, tau_min, tau_max, true, wrapToPi, 1.0);
}

void ControllerPID::configure(double kp, double ki, double kd, double tau_min, double tau_max, bool delta_implementation, bool wrapToPi) {
  configure(kp, ki, kd, 0.0, 0.0, 0.0, tau_min, tau_max, delta_implementation, wrapToPi, 1.0);
}
void ControllerPID::configure(double kp, double ki, double kd, double tau_min, double tau_max, bool wrapToPi) {
  configure(kp, ki, kd, 0.0, 0.0, 0.0, tau_min, tau_max, true, wrapToPi, 1.0);
}

double ControllerPID::differentiateState(double state, double dt) {
  if (dt <= 0.0) {
    return 0.0;
  }

  const double state_delta = wrapToPi_ ? farol2_utils::wrapToPi(state - state_prev_) : (state - state_prev_);

  if (first_it_) {
    state_rate_diff_prev_ = 0.0;
    return 0.0;
  }

  // Tustin discretization of H(s) = w*s/(s + w):
  // y[k] = alpha*y[k-1] + beta*(x[k]-x[k-1]),
  // alpha = (2 - w*dt)/(2 + w*dt), beta = 2*w/(2 + w*dt).
  const double denom = 2.0 + state_rate_diff_wc_ * dt;
  const double alpha = (2.0 - state_rate_diff_wc_ * dt) / denom;
  const double beta = (2.0 * state_rate_diff_wc_) / denom;
  const double state_rate = alpha * state_rate_diff_prev_ + beta * state_delta;
  state_rate_diff_prev_ = state_rate;
  return state_rate;
}

double ControllerPID::callController(double state, double state_ref, double dref, double ddref, double dt) {
  const double state_rate = differentiateState(state, dt);
  return callControllerImpl(state, state_ref, state_rate, dref, ddref, dt);
}

double ControllerPID::callController(double state, double state_ref, double state_rate,
                                     double dref, double ddref, double dt) {
  return callControllerImpl(state, state_ref, state_rate, dref, ddref, dt);
}

// Delta implementation for PID
double ControllerPID::callControllerImpl(double state, double state_ref, double state_rate,
                                         double dref, double ddref, double dt) {
  if (!configured_ || dt <= 0.0) return 0.0;
  state_rate_raw_ = state_rate;
  state_ = state;
  ref_ = state_ref;
  dref_ = dref;
  ddref_ = ddref;

  const double state_for_control = state;
  const double ref_for_control = ref_;
  const double state_rate_used = state_rate;

  state_used_for_control_ = state_for_control;
  ref_used_for_control_ = ref_for_control;
  state_rate_used_ = state_rate_used;

  //////////////  Actual pid computation  //////////////

  // Compute error and error rate
  error_ = state_for_control - ref_for_control;
  if (wrapToPi_)  // Wrap to [-pi, pi] if needed
    error_ = farol2_utils::wrapToPi(error_);
  // error_rate_ = state_rate_used_ - dref_;
  error_rate_ =  0.0- dref_;
  
  // Manual derivaties of everything because delta implementation
  error_dot_ = (error_ - error_prev_) / dt; // this is outside so we dont miss the initial step so that delta behaves more like tradition pid
  if (!first_it_) {
    // Compute derivative of all terms except the integral
    state_rate_dot_ = (state_rate_used - state_rate_prev_) / dt;
    state_dot_ = state_rate_used;
    error_rate_dot_ = (error_rate_ - error_rate_prev_) / dt;
    ddref_dot_ = (ddref_ - ddref_prev_) / dt;
  } else
    first_it_ = false;  // Reset first iteration flag

  ////     PID using Delta Implementation   ////
  if(delta_implementation_){
    // Add all PID terms + drag compensation
    tau_d_ = -ki_ * error_ - kp_ * error_dot_ - kd_ * error_rate_dot_ + kffv_lin_ * state_rate_dot_ + kffv_sq_ * state_rate_dot_ * abs(state_rate_dot_) + kffa_*ddref_dot_;

    // for debug only
    p_term_ = -kp_ * error_dot_*dt;
    i_term_ = -ki_ * error_*dt;
    d_term_ = -kd_ * error_rate_dot_ + kffv_lin_ * state_rate_dot_ + kffv_sq_ * state_rate_dot_ * abs(state_rate_dot_)*dt;

    // Anti-windup 
    Ka_ = 1.0 / dt;
    tau_dot_ = tau_d_ - Ka_ * (tau_prev_ - tau_sat_prev_);
    tau_ = tau_prev_ + tau_dot_ * dt;
    tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);
  }
  ////    Traditional PID with integral anti-windup   ////
  else{
    // integral term with anti-windup
    tau_d_ = -ki_*error_;
    Ka_ = 1.0/dt;
    tau_dot_ = tau_d_ - Ka_*(tau_prev_ - tau_sat_prev_);
    if(abs(tau_)<tau_max_/2)
      i_term_ = i_term_ + tau_dot_*dt ;
    
    p_term_ = -kp_ * error_;
    d_term_ = -kd_ * error_rate_;
    
    // add all pid terms
    tau_ =  p_term_ + i_term_ + d_term_;

    std::cout << "P: " << p_term_ << " I: " << i_term_ << " D: " << d_term_ << std::endl;
    // antiwindup saturation
    tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);
  }

  // Set prev values 
  error_prev_ = error_;
  state_prev_ = state_for_control;
  state_rate_prev_ = state_rate_used;
  error_rate_prev_ = error_rate_;
  tau_prev_ = tau_;
  tau_sat_prev_ = tau_sat_;
  ddref_prev_ = ddref_;

  return tau_sat_;
}


void ControllerPID::setGains(double kp, double ki, double kd, double kffv_lin, double kffv_sq, double kffa) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
  i_term_ = 0.0; // reset integral term when gains are changed to avoid spikes
}

}  // namespace farol_control
