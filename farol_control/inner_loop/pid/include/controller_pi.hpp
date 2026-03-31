#ifndef CONTROLLER_PI_HPP_
#define CONTROLLER_PI_HPP_

/**
 * @file controller_pi.hpp
 * @brief PI (Proportional-Integral) controller with anti-windup and low-pass filtering
 * @author Ravi Regalo
 * @author Eduardo Cunha
 */

namespace farol_control {

/**
 * @class ControllerPI
 * @brief Proportional-Integral controller with saturation and anti-windup
 * 
 * Implements a discrete-time PI controller with:
 * - Anti-windup via back-calculation
 * - Input rate low-pass filtering for derivative computation
 * - Output saturation bounds
 */
class ControllerPI {
 public:
  /**
   * @brief Constructor
   * @param kp Proportional gain
   * @param ki Integral gain  
   * @param lpf_wc Low-pass filter cutoff frequency (rad/s) for state derivative smoothing
   * @param tau_min Minimum output saturation bound
   * @param tau_max Maximum output saturation bound
   */
  ControllerPI(double kp, double ki, double lpf_wc, double tau_min, double tau_max);

  /**
   * @brief Call controller with current state and reference
   * @param state Current measured state value
   * @param state_ref Desired reference state value
   * @param dt Time step in seconds
   * @return Saturated output control signal
   */
  double callController(double state, double state_ref, double dt);

  /**
   * @brief Update controller parameters
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param lpf_wc Low-pass filter cutoff frequency (rad/s)
   * @param tau_min Minimum output saturation bound
   * @param tau_max Maximum output saturation bound
   */
  void setParams(double kp, double ki, double lpf_wc, double tau_min, double tau_max);

  // Getter methods for diagnostics and debugging
  double getError() { return error_; }
  double getIntegralTerm() { return ki_ * error_; }
  double getProportionalTerm() { return kp_ * error_; }
  double getTau_d() { return tau_d_; }
  double getTau_sat() { return tau_sat_; }
  double getAntiWindupTerm() { return Ka_ * (tau_prev_ - tau_sat_prev_); }
  double getTauDot() { return tau_dot_; }
  double getTau() { return tau_; }

 private:
  // Controller parameters
  double kp_;        ///< Proportional gain
  double ki_;        ///< Integral gain
  double lpf_wc_;    ///< Low-pass filter cutoff frequency (rad/s)
  double tau_min_;   ///< Minimum output saturation bound
  double tau_max_;   ///< Maximum output saturation bound

  // Internal state variables
  double error_ = 0.0;
  double tau_d_ = 0.0;
  bool first_it_ = true;
  double state_prev_ = 0.0;
  double state_dot_ = 0.0;
  double state_dot_filter_ = 0.0;
  double state_dot_filter_prev_ = 0.0;
  double lpf_A_ = 0.0;
  double lpf_B_ = 0.0;
  double Ka_ = 0.0;
  double tau_dot_ = 0.0;
  double tau_ = 0.0;
  double tau_prev_ = 0.0;
  double tau_sat_ = 0.0;
  double tau_sat_prev_ = 0.0;
};

}  // namespace farol_control

#endif  // CONTROLLER_PI_HPP_
