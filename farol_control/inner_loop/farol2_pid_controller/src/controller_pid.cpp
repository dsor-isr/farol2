#include "farol2_pid_controller/controller_pid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <farol2_utils/angles.hpp>

namespace farol_control {

void ControllerPID::configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq,
                              double kffa, double tau_min, double tau_max,
                              bool delta_implementation, bool wrapToPi) {
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
  configured_ = true;
}

void ControllerPID::configure(double kp, double ki, double kd, double kffv_lin, double kffv_sq,
                              double kffa, double tau_min, double tau_max, bool wrapToPi) {
  configure(kp, ki, kd, kffv_lin, kffv_sq, kffa, tau_min, tau_max, true, wrapToPi);
}

// Delta implementation for PID
double ControllerPID::callController(double state, double state_ref, double state_rate,
                                     double dref, double ddref, double dt) {
  if (!configured_) return 0.0;
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
  error_rate_ = state_rate - dref_;  // only used for regular pid implementation. delta adds it after antiwindup 
  
  error_dot_ = (error_ - error_prev_) / dt; // this is outside so we dont miss the initial step so that delta behaves more like tradition pid
  if (!first_it_) {
    // Compute derivative of all terms except the integral
    state_rate_dot_ = (state_rate_used - state_rate_prev_) / dt;
    state_dot_ = wrapToPi_ ? farol2_utils::wrapToPi(state_for_control - state_prev_) / dt : (state_for_control - state_prev_) / dt;
    error_rate_dot_ = (error_rate_ - error_rate_prev_) / dt;
    ddref_dot_ = (ddref_ - ddref_prev_) / dt;
  } else
    first_it_ = false;  // Reset first iteration flag

  ////     PID using Delta Implementation   ////
  if(delta_implementation_){
    // Add all PID terms + drag compensation
    tau_d_ = -ki_ * error_ - kp_ * error_dot_ - kd_ * error_rate_dot_ + kffv_lin_ * state_rate_dot_ + kffv_sq_ * state_rate_dot_ * abs(state_rate_dot_);

    // for debug only
    p_term_ = -kp_ * error_dot_*dt;
    i_term_ = -ki_ * error_*dt;
    d_term_ = -kd_ * error_rate_dot_ + kffv_lin_ * state_rate_dot_ + kffv_sq_ * state_rate_dot_ * abs(state_rate_dot_)*dt;

    // Anti-windup 
    Ka_ = 1.0 / dt;
    tau_dot_ = tau_d_ - Ka_ * (tau_prev_ - tau_sat_prev_);
    tau_ = tau_prev_ + tau_dot_ * dt;
    tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);

    // add FF terms after antiwindup block
    // ff_term_ =  kd_ * dref_;// +  kffa_ * ddref_;
    // output_ = std::clamp(tau_sat_ + ff_term_, tau_min_, tau_max_);
    // output_ = tau_sat_;
  }
  ////    Traditional PID with integral anti-windup   ////
  else{
    ff_term_ = 0.0;
    // integral term with anti-windup
    tau_d_ = -ki_*error_;
    Ka_ = 1.0/dt;
    tau_dot_ = tau_d_ - Ka_*(tau_prev_ - tau_sat_prev_);
    i_term_ = i_term_ + tau_dot_*dt ;

    if(abs(tau_d_) < 0.0001) i_term_ = 0.0; // fix estupido
    p_term_ = -kp_ * error_;
    d_term_ = -kd_ * error_rate_;
    ff_term_ = 0.0;
    
    // add all pid terms
    tau_ =  p_term_ + i_term_ + d_term_ + ff_term_;
    
    // antiwindup saturation
    tau_sat_ = std::clamp(tau_, tau_min_, tau_max_);
    // output_ = tau_sat_;
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
}

}  // namespace farol_control
