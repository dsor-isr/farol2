#include <farol2_utils/filters/moving_average_filter.hpp>
#include <farol2_utils/angles.hpp>

#include <cmath>

namespace farol2_utils {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-12;
}

MovingAverageFilter::MovingAverageFilter() {
  reset();
}

void MovingAverageFilter::configure(std::size_t window_samples, bool wrap_angle, bool use_fixed_Ts, double Ts) {
  if (window_samples == 0U) {
    throw std::invalid_argument("window_samples must be >= 1");
  }
  if (use_fixed_Ts && Ts <= 0.0) {
    throw std::invalid_argument("Ts must be > 0");
  }

  mode_ = WindowMode::Samples;
  window_samples_ = window_samples;
  window_seconds_ = 0.0;
  wrap_angle_ = wrap_angle;
  use_fixed_Ts_ = use_fixed_Ts;
  Ts_ = Ts;

  configured_ = true;
  reset();
}

void MovingAverageFilter::configure(double window_seconds, bool wrap_angle, bool use_fixed_Ts, double Ts) {
  if (window_seconds <= 0.0) {
    throw std::invalid_argument("window_seconds must be > 0");
  }
  if (use_fixed_Ts && Ts <= 0.0) {
    throw std::invalid_argument("Ts must be > 0");
  }

  mode_ = WindowMode::Seconds;
  window_seconds_ = window_seconds;
  window_samples_ = 1;
  wrap_angle_ = wrap_angle;
  use_fixed_Ts_ = use_fixed_Ts;
  Ts_ = Ts;

  configured_ = true;
  reset();
}

void MovingAverageFilter::reset(double y0) {
  samples_unwrapped_.clear();
  sample_dts_.clear();
  weighted_sum_ = 0.0;
  total_time_ = 0.0;
  y_unwrapped_ = y0;
  have_unwrapped_input_ = false;
}

double MovingAverageFilter::unwrapToReference(double x, double ref) {
  while (x - ref > kPi) {
    x -= 2.0 * kPi;
  }
  while (x - ref < -kPi) {
    x += 2.0 * kPi;
  }
  return x;
}

void MovingAverageFilter::step(double x) {
  if (!configured_) {
    throw std::runtime_error("MovingAverageFilter: call configure*() before step()");
  }

  if (mode_ == WindowMode::Seconds && !use_fixed_Ts_) {
    throw std::runtime_error("MovingAverageFilter: step(x) requires use_fixed_Ts=true for time-window mode");
  }

  const double dt_used = use_fixed_Ts_ ? Ts_ : 0.0;
  step(x, dt_used);
}

void MovingAverageFilter::step(double x, double dt) {
  if (!configured_) {
    throw std::runtime_error("MovingAverageFilter: call configure*() before step()");
  }

  const double dt_used = use_fixed_Ts_ ? Ts_ : dt;

  if (mode_ == WindowMode::Seconds && dt_used <= 0.0) {
    throw std::invalid_argument("dt must be > 0");
  }

  double x_unwrapped = x;
  if (wrap_angle_) {
    if (!have_unwrapped_input_) {
      have_unwrapped_input_ = true;
    } else {
      x_unwrapped = unwrapToReference(x, samples_unwrapped_.back());
    }
  }

  pushSample(x_unwrapped, dt_used);
  updateOutput();
}

void MovingAverageFilter::pushSample(double x_unwrapped, double dt_used) {
  if (mode_ == WindowMode::Samples) {
    samples_unwrapped_.push_back(x_unwrapped);
    weighted_sum_ += x_unwrapped;

    while (samples_unwrapped_.size() > window_samples_) {
      weighted_sum_ -= samples_unwrapped_.front();
      samples_unwrapped_.pop_front();
    }
    return;
  }

  samples_unwrapped_.push_back(x_unwrapped);
  sample_dts_.push_back(dt_used);
  weighted_sum_ += x_unwrapped * dt_used;
  total_time_ += dt_used;

  while (!sample_dts_.empty() && total_time_ - window_seconds_ > kEps) {
    const double excess = total_time_ - window_seconds_;
    const double dt_front = sample_dts_.front();

    if (dt_front <= excess + kEps) {
      weighted_sum_ -= samples_unwrapped_.front() * dt_front;
      total_time_ -= dt_front;
      sample_dts_.pop_front();
      samples_unwrapped_.pop_front();
    } else {
      sample_dts_.front() -= excess;
      weighted_sum_ -= samples_unwrapped_.front() * excess;
      total_time_ = window_seconds_;
    }
  }
}

void MovingAverageFilter::updateOutput() {
  if (samples_unwrapped_.empty()) {
    y_unwrapped_ = 0.0;
    return;
  }

  if (mode_ == WindowMode::Samples) {
    y_unwrapped_ = weighted_sum_ / static_cast<double>(samples_unwrapped_.size());
    return;
  }

  if (total_time_ <= kEps) {
    y_unwrapped_ = samples_unwrapped_.back();
    return;
  }

  y_unwrapped_ = weighted_sum_ / total_time_;
}

double MovingAverageFilter::y() const {
  if (!wrap_angle_) {
    return y_unwrapped_;
  }
  return wrapTo2Pi(y_unwrapped_);
}

} // namespace farol2_utils
