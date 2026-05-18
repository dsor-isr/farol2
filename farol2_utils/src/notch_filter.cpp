#include <farol2_utils/filters/notch_filter.hpp>

#include <cmath>

namespace farol2_utils {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

NotchFilter::NotchFilter() {
  reset();
}

void NotchFilter::configure(double f0, double Q, bool use_fixed_Ts, double Ts) {
  if (f0 <= 0.0) {
    throw std::invalid_argument("f0 must be > 0");
  }
  if (Q <= 0.0) {
    throw std::invalid_argument("Q must be > 0");
  }
  if (use_fixed_Ts && Ts <= 0.0) {
    throw std::invalid_argument("Ts must be > 0");
  }

  f0_ = f0;
  Q_ = Q;
  use_fixed_Ts_ = use_fixed_Ts;
  Ts_ = Ts;
  configured_ = true;
  reset();
}

void NotchFilter::reset() {
  x1_ = 0.0;
  x2_ = 0.0;
  y1_ = 0.0;
  y2_ = 0.0;
  y_ = 0.0;
}

void NotchFilter::step(double x) {
  if (!use_fixed_Ts_) {
    throw std::runtime_error("NotchFilter: step(x) requires use_fixed_Ts=true");
  }
  step(x, Ts_);
}

void NotchFilter::step(double x, double dt) {
  if (!configured_) {
    throw std::runtime_error("NotchFilter: call configure*() before step()");
  }

  const double fs = 1.0 / dt;
  if (fs <= 0.0) {
    throw std::invalid_argument("fs must be > 0");
  }
  if (f0_ >= fs / 2.0) {
    throw std::invalid_argument("f0 must be < fs/2");
  }

  updateCoefficients(fs);

  const double yk = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;

  x2_ = x1_;
  x1_ = x;
  y2_ = y1_;
  y1_ = yk;
  y_ = yk;
}

double NotchFilter::y() const {
  return y_;
}

void NotchFilter::updateCoefficients(double fs) {
  fs_ = fs;

  const double w0 = 2.0 * kPi * f0_ / fs_;
  const double alpha = std::sin(w0) / (2.0 * Q_);
  const double c = std::cos(w0);

  const double b0 = 1.0;
  const double b1 = -2.0 * c;
  const double b2 = 1.0;
  const double a0 = 1.0 + alpha;
  const double a1 = -2.0 * c;
  const double a2 = 1.0 - alpha;

  b0_ = b0 / a0;
  b1_ = b1 / a0;
  b2_ = b2 / a0;
  a1_ = a1 / a0;
  a2_ = a2 / a0;
}

} // namespace farol2_utils
