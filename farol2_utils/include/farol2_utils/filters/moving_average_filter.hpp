#pragma once

#include <cstddef>
#include <deque>
#include <stdexcept>

namespace farol2_utils {

class MovingAverageFilter {
public:
  MovingAverageFilter();

  // Configure moving average by number of samples.
  // If use_fixed_Ts is false, runtime dt passed to step() is used when calling step(x, dt).
  // If use_fixed_Ts is true, Ts is always used and step dt is ignored.
  void configure(std::size_t window_samples, bool wrap_angle=false, bool use_fixed_Ts=false, double Ts=0.0);

  // Configure moving average by time window in seconds.
  // If use_fixed_Ts is false, runtime dt passed to step() is required.
  // If use_fixed_Ts is true, Ts is always used and step dt is ignored.
  void configure(double window_seconds, bool wrap_angle=false, bool use_fixed_Ts=false, double Ts=0.0);

  // Reset internal histories.
  void reset(double y0 = 0.0);

  // Step using configured fixed Ts (requires use_fixed_Ts=true for time window mode).
  void step(double x);

  // Causal sample-by-sample update with runtime sampling period.
  void step(double x, double dt);

  double y() const;

  bool isConfigured() const { return configured_; }

private:
  enum class WindowMode {
    Samples,
    Seconds
  };

  static double unwrapToReference(double x, double ref);

  void pushSample(double x_unwrapped, double dt_used);
  void updateOutput();

  bool configured_{false};
  bool wrap_angle_{false};
  bool use_fixed_Ts_{false};
  bool have_unwrapped_input_{false};

  WindowMode mode_{WindowMode::Samples};

  std::size_t window_samples_{1};
  double window_seconds_{0.0};
  double Ts_{0.0};

  double weighted_sum_{0.0};
  double total_time_{0.0};
  double y_unwrapped_{0.0};

  std::deque<double> samples_unwrapped_;
  std::deque<double> sample_dts_;
};

} // namespace farol2_utils
