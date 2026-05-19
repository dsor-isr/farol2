#ifndef CONTROLLER_PI_HPP_
#define CONTROLLER_PI_HPP_

namespace farol_control {

class ControllerPI {
 public:
  ControllerPI() = default;

  void configure(double kp, double ki, double tau_min, double tau_max, bool use_lpf = true, double lpf_wc = 1.0);

  double callController(double state, double state_ref, double dt);

  void setGains(double kp, double ki);

  double getError() { return error_; }
  double getIntegralTerm() { return ki_ * error_; }
  double getProportionalTerm() { return kp_ * error_; }
  double getTau_d() { return tau_d_; }
  double getTau_sat() { return tau_sat_; }
  double getAntiWindupTerm() { return Ka_ * (tau_prev_ - tau_sat_prev_); }
  double getTauDot() { return tau_dot_; }
  double getTau() { return tau_; }

 private:
  double kp_;
  double ki_;
  double lpf_wc_;
  double tau_min_;
  double tau_max_;
  bool use_lpf_;
  bool configured_ = false;

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