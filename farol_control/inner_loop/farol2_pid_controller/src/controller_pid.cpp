#include "farol2_pid_controller/controller_pid.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <farol2_utils/angles.hpp>

namespace farol_control {

void ControllerPID::configure(double kp, double ki, double kd, double kdff, double kffv_lin, double kffv_sq,
                              double kffa, double tau_min, double tau_max, bool use_ref_lpf,
                              double lpf_wc, int lpf_order, std::string lpf_design,
                              std::string lpf_method, bool delta_implementation, bool wrapToPi,
                              bool use_state_lpf, bool use_state_lpf_for_state_rate,
                              bool use_filtered_state_for_control,
                              bool use_filtered_ref_for_control,
                              bool use_rate_limiter,
                              double rate_limit) {
  /* Set parameters */
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kdff_ = kdff;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
  lpf_wc_ = (lpf_wc > 0.0) ? lpf_wc : 1.0;
  tau_min_ = tau_min;
  tau_max_ = tau_max;
  wrapToPi_ = wrapToPi;
  use_ref_lpf_ = use_ref_lpf;
  use_state_lpf_ = use_state_lpf;
  use_state_lpf_for_state_rate_ = use_state_lpf_for_state_rate;
  use_filtered_state_for_control_ = use_filtered_state_for_control;
  use_filtered_ref_for_control_ = use_filtered_ref_for_control;
  use_rate_limiter_ = use_rate_limiter;
  rate_limit_ = std::max(0.0, rate_limit);
  if(wrapToPi) rate_limit_ = farol2_utils::deg2rad(rate_limit_);                            
  rate_limiter_initialized_ = false;
  ref_rate_limited_ = 0.0;
  delta_implementation_ = delta_implementation;

  // If any feature needs the state LPF output, ensure the LPF is active.
  use_state_lpf_ = use_state_lpf_ || use_state_lpf_for_state_rate_ || use_filtered_state_for_control_;

  lpf_.configure(lpf_wc_, 0.1, lpf_order, lpf_design, lpf_method, wrapToPi_);
  state_lpf_.configure(lpf_wc_, 0.1, lpf_order, lpf_design, lpf_method, wrapToPi_);
  configured_ = true;
}

void ControllerPID::configure(double kp, double ki, double kd, double kdff, double kffv_lin, double kffv_sq,
                              double kffa, double tau_min, double tau_max, bool use_ref_lpf,
                              double lpf_wc, int lpf_order, std::string lpf_design,
                              std::string lpf_method, bool wrapToPi) {
  configure(kp, ki, kd, kdff, kffv_lin, kffv_sq, kffa, tau_min, tau_max, use_ref_lpf, lpf_wc,
            lpf_order, lpf_design, lpf_method, true, wrapToPi, false, false, false, false, false, 0.0);
}

// Delta implementation for PID
double ControllerPID::callController(double state, double state_ref, double state_rate, double dt) {
  // make sure ref is smooth
  if (first_it_) {
    state_ref = state;
    lpf_.reset(state_ref);
    state_lpf_.reset(state);
  }

  if (!configured_) return 0.0;
  ref_raw_ = state_ref;
  state_raw_ = state;
  state_rate_raw_ = state_rate;
  state_ = state;

  
  // Aply a rate limiter to the reference to make it feasisble for the controller to track position and velocity errors
  double limited_ref = state_ref;
  if (use_rate_limiter_ && rate_limit_ > 0.0 && dt > 0.0) {
    if (!rate_limiter_initialized_) {
      ref_rate_limited_ = state_ref;
      rate_limiter_initialized_ = true;
    } else {
      double delta_ref = state_ref - ref_rate_limited_;
      if (wrapToPi_) {
        delta_ref = farol2_utils::wrapToPi(delta_ref);
      }
      
      const double max_step = rate_limit_ * dt;
      delta_ref = std::clamp(delta_ref, -max_step, max_step);
      ref_rate_limited_ += delta_ref;
      
      if (wrapToPi_) {
        ref_rate_limited_ = farol2_utils::wrapToPi(ref_rate_limited_);
      }
    }
    limited_ref = ref_rate_limited_;
  } else {
    rate_limiter_initialized_ = false;
    ref_rate_limited_ = state_ref;
  }
  
  // Pass reference signal through LPF to extract reference derivatives for ff terms
  if (use_ref_lpf_) {
    lpf_.step(limited_ref, dt);
    ref_ = lpf_.y();
    dref_ = lpf_.dy();
    ddref_ = lpf_.ddy();
  } else {
    ref_ = limited_ref;
    dref_ = 0.0;
    ddref_ = 0.0;
    dddref_ = 0.0;
  }
  
  // Pass state through LPF to extract state in case it is not measured
  double dstate_from_lpf = 0.0;
  if (use_state_lpf_) {
    state_lpf_.step(state, dt);
    state_filt_ = state_lpf_.y();
    dstate_from_lpf = state_lpf_.dy();
  } else {
    state_filt_ = state;
  }
  state_rate_lpf_ = dstate_from_lpf;

  const double state_for_control = use_filtered_state_for_control_ ? state_filt_ : state;
  const double ref_for_control = use_filtered_ref_for_control_ ? ref_ : limited_ref;
  const double state_rate_used = use_state_lpf_for_state_rate_ ? dstate_from_lpf : state_rate;
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
    // ff_term_ =  kdff_ *kd_ * dref_;// +  kffa_ * ddref_;
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
    ff_term_ = kdff_* dref_;
    
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
  state_rate_dot_filter_prev_ = state_rate_dot_filter_;
  tau_prev_ = tau_;
  tau_sat_prev_ = tau_sat_;
  ddref_prev_ = ddref_;

  return tau_sat_;
}


void ControllerPID::setGains(double kp, double ki, double kd, double kdff, double kffv_lin, double kffv_sq, double kffa) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  kdff_ = kdff;
  kffv_lin_ = kffv_lin;
  kffv_sq_ = kffv_sq;
  kffa_ = kffa;
}

}  // namespace farol_control
