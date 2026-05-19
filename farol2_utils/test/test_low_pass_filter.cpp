#include <farol2_utils/filters/low_pass_filter.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

double mean(const std::vector<double> &values) {
  if (values.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (double v : values) {
    sum += v;
  }
  return sum / static_cast<double>(values.size());
}

double stddev(const std::vector<double> &values) {
  if (values.size() < 2) {
    return 0.0;
  }

  const double m = mean(values);
  double acc = 0.0;
  for (double v : values) {
    const double d = v - m;
    acc += d * d;
  }
  return std::sqrt(acc / static_cast<double>(values.size()));
}

}  // namespace

TEST(LowPassFilterTest, RejectsInvalidUsage) {
  farol2_utils::LowPassFilter lpf;

  EXPECT_THROW(lpf.step(0.0, 0.01), std::runtime_error);
  EXPECT_THROW(lpf.configure(0.0, 0.1, 3, "bessel", "tustin", false), std::invalid_argument);
  EXPECT_THROW(lpf.configure(1.0, 0.1, 0, std::string("bessel"), std::string("tustin"), false), std::invalid_argument);
}

TEST(LowPassFilterTest, FixedStepRampHasSmallDdrefNoiseAfterTransient) {
  farol2_utils::LowPassFilter lpf;
  const double dt = 0.1;
  const double slope = 0.07;

  lpf.configure(6.28, 3, std::string("bessel"), std::string("tustin"), false, true, 0.1);
  lpf.reset(0.0, 0.0);

  std::vector<double> dd_tail;
  dd_tail.reserve(400);

  for (int k = 0; k < 800; ++k) {
    const double t = static_cast<double>(k) * dt;
    const double ref = slope * t;
    lpf.step(ref);

    if (k >= 400) {
      dd_tail.push_back(lpf.ddy());
    }
  }

  EXPECT_LT(std::abs(mean(dd_tail)), 2e-2);
  EXPECT_LT(stddev(dd_tail), 2e-2);
}

TEST(LowPassFilterTest, FixedStepMismatchIncreasesDdrefNoiseUnderJitteredSampling) {
  farol2_utils::LowPassFilter lpf_fixed;
  farol2_utils::LowPassFilter lpf_var;

  const double dt_nominal = 0.1;
  const double slope = 0.07;

  lpf_fixed.configure(6.28, 3, std::string("bessel"), std::string("tustin"), false, true, 0.1);
  lpf_var.configure(6.28, 3, std::string("bessel"), std::string("tustin"), false, false, 0.0);
  lpf_fixed.reset(0.0, 0.0);
  lpf_var.reset(0.0, 0.0);

  std::vector<double> dd_fixed_tail;
  std::vector<double> dd_var_tail;
  dd_fixed_tail.reserve(400);
  dd_var_tail.reserve(400);

  double t = 0.0;
  for (int k = 0; k < 900; ++k) {
    const double jitter = 0.015 * std::sin(0.13 * static_cast<double>(k));
    const double dt = dt_nominal + jitter;
    t += dt;
    const double ref = slope * t;

    lpf_fixed.step(ref);
    lpf_var.step(ref, dt);

    if (k >= 500) {
      dd_fixed_tail.push_back(lpf_fixed.ddy());
      dd_var_tail.push_back(lpf_var.ddy());
    }
  }

  const double fixed_std = stddev(dd_fixed_tail);
  const double var_std = stddev(dd_var_tail);

  EXPECT_LT(var_std, fixed_std * 0.5);
}
