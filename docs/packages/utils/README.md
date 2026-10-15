# farol2_utils

A C++ library (no nodes) of small math, signal-processing and TF helpers. Used by allocation, console, inner_loop, nav, path_following, planning and waypoint. Everything is in namespace `farol2_utils`; errors are reported with `std::invalid_argument` and `std::runtime_error`.

## Contents

| Header | What it provides | Used by |
|---|---|---|
| `angles.hpp` | `pi_v<T>`, `deg2rad`, `rad2deg`, `wrapTo2Pi` ([0, 2π)), `wrapToPi` ([−π, π)). Header-only templates | most packages |
| `filters/low_pass_filter.hpp` | `LowPassFilter`: state-space low-pass of order 1 to N, Butterworth or Bessel design, Euler, ZOH or Tustin discretisation, outputs `y`, `dy`, `ddy`, optional angle wrapping. See [low_pass_filter.md](low_pass_filter.md) | inner_loop reference generator |
| `filters/notch_filter.hpp` | `NotchFilter`: 2nd-order IIR notch at `f0` with quality `Q`, Nyquist guard. See [notch_filter.md](notch_filter.md) | (none in the stack yet) |
| `filters/moving_average_filter.hpp` | `MovingAverageFilter`: window in samples or seconds, angle-aware (unwraps before averaging) | nav `yaw_rate_ekf` measurement windows |
| `tf_utils.hpp` | `StaticTransformLookup`: caches static transforms as `Eigen::Isometry3d` | nav `sample_and_hold` |

### Filter API pattern

All three filters share the same shape:

```cpp
filter.configure(..., bool use_fixed_Ts = false, double Ts = 0.0);
filter.reset(...);
filter.step(x);        // uses the fixed Ts
filter.step(x, dt);    // uses dt unless use_fixed_Ts
double y = filter.y();
```

```cpp
#include <farol2_utils/filters/low_pass_filter.hpp>

farol2_utils::LowPassFilter lpf;
lpf.configure(2.0 /*wc rad/s*/, 0.1 /*Ts*/, 2, "butterworth", "tustin");
for (double u : signal) {
  lpf.step(u, 0.1);
  double y = lpf.y(), dy = lpf.dy();
}
```

### StaticTransformLookup

`lookup(target, source)` returns the cached transform `target ← source`. Empty frames or `target == source` give identity. **A transform that is not in TF also gives identity**, with one warning per frame pair, and is retried on the next call. This is why vehicles without sensor frames in their URDF run with zero lever arms and no rotation ([GAPS #24](../../GAPS.md#24-only-myellow-defines-sensor-frames-low)). `clear()` drops the cache.

## Using it from another package

```cmake
find_package(farol2_utils REQUIRED)
ament_target_dependencies(my_target farol2_utils)   # as the FAROL 2 packages do
# or: target_link_libraries(my_target farol2_utils::farol2_utils)
```

Add `<depend>farol2_utils</depend>` to `package.xml`.

The library exports `Eigen3`, `rclcpp`, `tf2`, `tf2_eigen` and `tf2_ros`.

## Tests

`colcon test --packages-select farol2_utils` runs three gtest suites: `test_low_pass_filter`, `test_notch_filter` and `test_moving_average_filter`. One notch test fails on `dev` ([GAPS #20](../../GAPS.md#20-a-unit-test-fails-on-dev-medium)). These are the only unit tests in the repository ([GAPS #27](../../GAPS.md#27-test-coverage-low)).
