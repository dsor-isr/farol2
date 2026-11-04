# farol2_utils

C++ utility library used by the nav, inner loop, allocation, waypoint and sim packages (target `farol2_utils`, depends on Eigen3, `tf2_ros`, `tf2_eigen`).

| Header | Content | Doc |
|---|---|---|
| `farol2_utils/angles.hpp` | angle conversion and wrapping (`deg2rad`, `rad2deg`, `wrapToPi`, `wrapTo2Pi`) | [farol_utils](docs/farol_utils.md) |
| `farol2_utils/filters/` | low-pass filter (Butterworth / Bessel; Euler, ZOH, Tustin), notch filter, moving average filter | [low_pass_filter](docs/low_pass_filter.md), [notch_filter](docs/notch_filter.md) |
| `farol2_utils/tf_utils.hpp` | `StaticTransformLookup(node)`: `lookup(target_frame, source_frame)` returns an `Eigen::Isometry3d` from `tf_static`, caches it, and returns identity with a one-time warning if the transform is missing (this is the `Static transform ... is not available, using identity` warning seen at start-up) | |

The package has a gtest suite for the filters (`colcon test --packages-select farol2_utils`).
