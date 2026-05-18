#include <farol2_utils/filters/notch_filter.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

double computeRms(const std::vector<double> &v) {
  if (v.empty()) {
    return 0.0;
  }

  double sum_sq = 0.0;
  for (double x : v) {
    sum_sq += x * x;
  }

  return std::sqrt(sum_sq / static_cast<double>(v.size()));
}

} // namespace

TEST(NotchFilterTest, RejectsInvalidParametersAndUsage) {
  farol2_utils::NotchFilter nf;

  EXPECT_THROW(nf.step(0.0, 0.01), std::runtime_error);
  EXPECT_THROW(nf.configure(0.0, 5.0), std::invalid_argument);
  EXPECT_THROW(nf.configure(-1.0, 5.0), std::invalid_argument);
  EXPECT_THROW(nf.configure(5.0, 0.0), std::invalid_argument);
  EXPECT_THROW(nf.configure(5.0, -1.0), std::invalid_argument);

  nf.configure(10.0, 5.0);
  EXPECT_THROW(nf.step(0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(nf.step(0.0, -0.01), std::invalid_argument);

  // fs = 1 / dt = 20 Hz -> fs/2 = 10 Hz, must satisfy f0 < fs/2.
  EXPECT_THROW(nf.step(0.0, 0.05), std::invalid_argument);
}

TEST(NotchFilterTest, ConstantInputApproximatelyPreserved) {
  farol2_utils::NotchFilter nf;
  const double fs = 100.0;
  const double dt = 1.0 / fs;

  nf.configure(10.0, 5.0);

  std::vector<double> tail;
  tail.reserve(1000);

  for (int k = 0; k < 4000; ++k) {
    nf.step(1.0, dt);
    if (k >= 3000) {
      tail.push_back(nf.y());
    }
  }

  double mean = 0.0;
  for (double y : tail) {
    mean += y;
  }
  mean /= static_cast<double>(tail.size());

  EXPECT_NEAR(mean, 1.0, 1e-2);
}

TEST(NotchFilterTest, SineAtNotchFrequencyIsAttenuated) {
  farol2_utils::NotchFilter nf;
  const double fs = 200.0;
  const double dt = 1.0 / fs;
  const double f0 = 25.0;

  nf.configure(f0, 8.0);

  std::vector<double> in;
  std::vector<double> out;
  in.reserve(4000);
  out.reserve(4000);

  for (int k = 0; k < 5000; ++k) {
    const double t = static_cast<double>(k) * dt;
    const double x = std::sin(2.0 * kPi * f0 * t);
    nf.step(x, dt);

    if (k >= 1000) {
      in.push_back(x);
      out.push_back(nf.y());
    }
  }

  const double rms_in = computeRms(in);
  const double rms_out = computeRms(out);

  ASSERT_GT(rms_in, 0.1);
  EXPECT_LT(rms_out / rms_in, 0.2);
}

TEST(NotchFilterTest, SineFarFromNotchIsMostlyPreserved) {
  farol2_utils::NotchFilter nf;
  const double fs = 200.0;
  const double dt = 1.0 / fs;
  const double f0 = 25.0;
  const double f_pass = 5.0;

  nf.configure(f0, 8.0);

  std::vector<double> in;
  std::vector<double> out;
  in.reserve(4000);
  out.reserve(4000);

  for (int k = 0; k < 5000; ++k) {
    const double t = static_cast<double>(k) * dt;
    const double x = std::sin(2.0 * kPi * f_pass * t);
    nf.step(x, dt);

    if (k >= 1000) {
      in.push_back(x);
      out.push_back(nf.y());
    }
  }

  const double rms_in = computeRms(in);
  const double rms_out = computeRms(out);
  const double ratio = rms_out / rms_in;

  ASSERT_GT(rms_in, 0.1);
  EXPECT_GT(ratio, 0.8);
  EXPECT_LT(ratio, 1.2);
}

TEST(NotchFilterTest, ResetClearsMemory) {
  farol2_utils::NotchFilter nf;
  const double fs = 100.0;
  const double dt = 1.0 / fs;

  nf.configure(10.0, 6.0);

  for (int k = 0; k < 500; ++k) {
    nf.step(1.0, dt);
  }

  nf.reset();

  for (int k = 0; k < 100; ++k) {
    nf.step(0.0, dt);
    EXPECT_NEAR(nf.y(), 0.0, 1e-12);
  }
}
