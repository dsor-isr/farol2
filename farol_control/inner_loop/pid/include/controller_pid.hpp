#ifndef CONTROLLER_PID_HPP_
#define CONTROLLER_PID_HPP_

#include <string>
#include <farol_utils/filters/low_pass_filter.hpp>

/**
 * @file controller_pid.hpp
 * @brief PID (Proportional-Integral-Derivative) controller with feed-forward and filtering
 * @author Ravi Regalo
 */

namespace farol_control {

/**
 * @class ControllerPID
 * @brief Advanced PID controller with derivative filtering, feed-forward terms, and anti-windup
 * 
 * Implements a discrete-time PID controller with:
 * - Reference signal filtering with derivative extraction for feed-forward lookahead
 * - Angle wrapping support for cyclic states (e.g., heading control)
 * - Feed-forward acceleration term (linear and quadratic velocity-squared)
 * - Anti-windup via back-calculation
 * - Derivative action filtering
 * - Output saturation bounds
 */
class ControllerPID {
 public:
  /**
   * @brief Constructor
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param lpf_wc Low-pass filter cutoff frequency (rad/s) for reference filtering
   * @param tau_min Minimum output saturation bound
   * @param tau_max Maximum output saturation bound
   * @param kffv_lin Linear velocity feed-forward gain
   * @param kffv_sq Quadratic velocity squared feed-forward gain
   * @param kffa Acceleration feed-forward gain
   * @param wrapToPi If true, wrap error to [-pi, pi] for cyclic states
   * @param lpf_order Order of reference low-pass filter
   * @param lpf_method Method for LPF ("tustin", "zoh" or "euler")
   * @param lpf_design Design method for LPF ("bessel" or "butterworth")
   */
  ControllerPID(double kp, double ki, double kd, double lpf_wc, double tau_min, 
                double tau_max, double kffv_lin, double kffv_sq, double kffa, 
                bool wrapToPi, int lpf_order, std::string lpf_method, 
                std::string lpf_design);

  /**
   * @brief Call controller with current state, reference, and state rate
   * @param state Current measured state value
   * @param state_ref Desired reference state value
   * @param state_rate Current state rate (derivative of state)
   * @param dt Time step in seconds
   * @return Saturated output control signal
   */
  double callController(double state, double state_ref, double state_rate, double dt);

  /**
   * @brief Update controller parameters
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param lpf_wc Low-pass filter cutoff frequency (rad/s)
   * @param tau_min Minimum output saturation bound
   * @param tau_max Maximum output saturation bound
   * @param kffv_lin Linear velocity feed-forward gain
   * @param kffv_sq Quadratic velocity squared feed-forward gain
   * @param kffa Acceleration feed-forward gain
   */
  void setParams(double kp, double ki, double kd, double lpf_wc, double tau_min, 
                 double tau_max, double kffv_lin, double kffv_sq, double kffa);

  // Getter methods for diagnostics and debugging
  double getError() { return error_; }
  double getIntegralTerm() { return i_term_; }
  double getProportionalTerm() { return p_term_; }
  double getDerivativeTerm() { return d_term_; }
  double getTau_d() { return tau_d_; }
  double getTau_sat() { return tau_sat_; }
  double getAntiWindupTerm() { return Ka_ * (tau_prev_ - tau_sat_prev_); }
  double getTauDot() { return tau_dot_; }
  double getTau() { return tau_; }

  // Public state variables for debugging/monitoring (not recommended for external modification)
  double ref_raw_;              ///< Raw unfiltered reference input
  double state_;                ///< Current state value
  double ref_;                  ///< Filtered reference signal
  double dref_;                 ///< First derivative of filtered reference (for FF)
  double ddref_;                ///< Second derivative of filtered reference (for FF)
  double dddref_;               ///< Third derivative of filtered reference
  double kffv_lin_;             ///< Linear velocity feed-forward gain
  double kffv_sq_;              ///< Quadratic velocity feed-forward gain
  double kffa_;                 ///< Acceleration feed-forward gain
  double p_term_;               ///< Proportional term for debugging
  double i_term_;               ///< Integral term for debugging
  double d_term_;               ///< Derivative term for debugging
  double error_dot_;            ///< Time derivative of error for debugging
  double error_rate_dot_;       ///< Time derivative of error rate for debugging

  // Controller parameters
  double kp_;                   ///< Proportional gain
  double ki_;                   ///< Integral gain
  double kd_;                   ///< Derivative gain
  double lpf_wc_;               ///< Low-pass filter cutoff frequency (rad/s)
  double tau_min_;              ///< Minimum output saturation bound
  double tau_max_;              ///< Maximum output saturation bound
  bool wrapToPi_;               ///< Flag to wrap error to [-pi, pi]

  // Internal state for error computation and derivatives
  double error_ = 0.0;
  double error_rate_ = 0.0;
  double tau_d_ = 0.0;

  // Variables for discrete derivatives
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

  // Low-pass filter state (deprecated, to be removed)
  double state_rate_dot_filter_ = 0.0;
  double state_rate_dot_filter_prev_ = 0.0;
  double error_rate_dot_filter_ = 0.0;
  double error_rate_dot_filter_prev_ = 0.0;
  double lpf_A_ = 0.0;
  double lpf_B_ = 0.0;

  // Low-pass filter for reference signal
  farol_utils::LowPassFilter lpf_;

 private:
  // Private state and helper methods (if needed in future)
};

}  // namespace farol_control

#endif  // CONTROLLER_PID_HPP_
