#pragma once

#include <stdexcept>

namespace farol2_utils {

class NotchFilter {
public:
  NotchFilter();

  // Configure notch frequency [Hz], quality factor and Ts mode.
  // If use_fixed_Ts is false, runtime dt passed to step() is used.
  // If use_fixed_Ts is true, Ts is always used and step dt is ignored.
  void configure(double f0, double Q, bool use_fixed_Ts=false, double Ts=0.0);

  // Reset internal histories.
  void reset();

  // Step using configured fixed Ts (requires use_fixed_Ts=true).
  void step(double x);

  // Causal sample-by-sample update with runtime sampling period.
  void step(double x, double dt);

  double y() const;

  bool isConfigured() const { return configured_; }

private:
  void updateCoefficients(double fs);

  bool configured_{false};

  // Parameters
  double fs_{0.0};
  double f0_{0.0};
  double Q_{0.0};
  double Ts_{0.0};
  bool use_fixed_Ts_{false};

  // Normalized biquad coefficients
  double b0_{1.0};
  double b1_{0.0};
  double b2_{0.0};
  double a1_{0.0};
  double a2_{0.0};

  // State: previous inputs and outputs
  double x1_{0.0};
  double x2_{0.0};
  double y1_{0.0};
  double y2_{0.0};
  double y_{0.0};
};

} // namespace farol2_utils
