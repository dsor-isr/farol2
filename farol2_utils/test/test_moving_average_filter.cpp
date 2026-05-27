#include <farol2_utils/filters/moving_average_filter.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

TEST(MovingAverageFilterTest, RejectsInvalidUsage) {
  farol2_utils::MovingAverageFilter maf;

  EXPECT_THROW(maf.step(0.0, 0.01), std::runtime_error);
  EXPECT_THROW(maf.configure(static_cast<std::size_t>(0)), std::invalid_argument);
  EXPECT_THROW(maf.configure(0.0), std::invalid_argument);
  EXPECT_THROW(maf.configure(-1.0), std::invalid_argument);

  maf.configure(1.0, false, false);
  EXPECT_THROW(maf.step(0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(maf.step(0.0), std::runtime_error);

  EXPECT_THROW(maf.configure(static_cast<std::size_t>(5), false, true, 0.0), std::invalid_argument);
  EXPECT_THROW(maf.configure(1.0, false, true, 0.0), std::invalid_argument);
}

TEST(MovingAverageFilterTest, SamplesWindowComputesLastNSamplesAverage) {
  farol2_utils::MovingAverageFilter maf;
  maf.configure(static_cast<std::size_t>(3));

  maf.step(1.0);
  EXPECT_NEAR(maf.y(), 1.0, 1e-12);

  maf.step(2.0);
  EXPECT_NEAR(maf.y(), 1.5, 1e-12);

  maf.step(3.0);
  EXPECT_NEAR(maf.y(), 2.0, 1e-12);

  maf.step(4.0);
  EXPECT_NEAR(maf.y(), 3.0, 1e-12);
}

TEST(MovingAverageFilterTest, TimeWindowUsesWeightedAverageWithVariableDt) {
  farol2_utils::MovingAverageFilter maf;
  maf.configure(1.0, false, false);

  maf.step(1.0, 0.4);
  EXPECT_NEAR(maf.y(), 1.0, 1e-12);

  maf.step(3.0, 0.4);
  EXPECT_NEAR(maf.y(), 2.0, 1e-12);

  maf.step(5.0, 0.4);
  // Last 1.0 s: 0.2 s of 1.0 + 0.4 s of 3.0 + 0.4 s of 5.0
  EXPECT_NEAR(maf.y(), 3.4, 1e-12);
}

TEST(MovingAverageFilterTest, TimeWindowFixedTsAllowsStepWithoutDt) {
  farol2_utils::MovingAverageFilter maf;
  maf.configure(0.5, false, true, 0.1);

  for (int k = 0; k < 10; ++k) {
    maf.step(2.0);
  }

  EXPECT_NEAR(maf.y(), 2.0, 1e-12);
}

TEST(MovingAverageFilterTest, WrapAngleAveragesAcrossDiscontinuity) {
  farol2_utils::MovingAverageFilter maf;
  maf.configure(static_cast<std::size_t>(2), true);

  const double deg2rad = 3.14159265358979323846 / 180.0;
  maf.step(359.0 * deg2rad);
  maf.step(1.0 * deg2rad);

  EXPECT_NEAR(maf.y(), 0.0, 0.05);
}
