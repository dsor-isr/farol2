# farol2_utils

## Generic description
`farol2_utils` provides small, reusable math/signal-processing utilities used by multiple FAROL2 control and navigation nodes.

Main goals:
- deterministic runtime behavior
- low-overhead real-time calls
- simple APIs with explicit configuration and state reset
- reusable templates for angle conversion/wrapping

## Features included in package

### 1) Angle helpers (`include/farol2_utils/angles.hpp`)
- `pi_v<T>`: high-precision compile-time pi constant
- `deg2rad(T)`: degrees to radians
- `rad2deg(T)`: radians to degrees
- `wrapTo2Pi(T)`: wrap angle to `[0, 2*pi)`
- `wrapToPi(T)`: wrap angle to `[-pi, pi)`

These are header-only templates, no allocation, suitable for hot loops.

### Geometry helpers (`include/farol2_utils/geometry.hpp`)

These header-only Eigen helpers contain no ROS types or simulator state:

- `skew(v)`: cross-product matrix.
- `rotationIncrement(v)`: SO(3) exponential of an axis-angle rotation vector.
- `rotationFromRpy(rpy)` / `rpyFromRotation(R)`: RPY radians and rotation matrix conversions.
- `pointVelocity(v, omega, position)`: rigid-body velocity at an offset point, in the input frame.
- `forceToWrench(force, position_body, rotation_local_to_body)`: body force and moment from a local force.

Link the exported `farol2_utils::farol2_utils_geometry` target for this Eigen-only
interface. It does not link the ROS-dependent filters/TF library. The simulator's
ROS-free build uses `FAROL2_UTILS_GEOMETRY_ONLY=ON` when adding this package as a
subdirectory. State-dependent water/current selection and integration remain in
`farol2_sim`; generic helpers do not depend on that package.

### Validation helpers (`include/farol2_utils/validation.hpp`)

Shared header-only checks throw `std::invalid_argument` with the parameter name:

- `requireFinite`: a scalar or Eigen matrix/vector must contain finite values.
- `requireFiniteValues`: finite entries in a standard container.
- `requirePositive` / `requireNonnegative`: scalar sign and finiteness checks.
- `requireOrderedBounds` / `requireInRange`: finite inclusive bounds and range checks.
- `requireSize`, `requireNonEmpty`, `requireUnique`: standard container checks.
- `requireRotationMatrix`: a finite 3x3 proper rotation (SO(3)).
- `requirePositiveDefinite` / `requirePositiveSemidefinite`: finite, nonempty,
  square, symmetric real matrices with the requested eigenvalue signs.

The original matrix form `requirePositive(matrix, definite, name)` is also
available. Prefer the named matrix variants at call sites. Matrix checks accept
Eigen expressions and dynamic or fixed-size inputs. Positive definiteness is
strict; semidefinite checks allow eigenvalues down to `-tolerance` (default
`1e-10`). Symmetry uses Eigen's relative comparison with that tolerance. Rotation
checks default to `1e-8`. Both tolerances can be supplied explicitly.

```cpp
farol2_utils::requirePositive(dt, "integration dt");
farol2_utils::requirePositiveDefinite(inertia, "vehicle.inertia");
farol2_utils::requirePositiveSemidefinite(covariance, "sensor.covariance");
farol2_utils::requireInRange(latitude, -90.0, 90.0, "origin.latitude");
```

These checks use the same Eigen-only header interface as the geometry helpers.
Simulator-specific rules, such as allowed DOFs, actuator modes and limits that
must straddle zero, remain in the owning model. Incoming command callbacks keep
their existing reject-and-log behavior rather than throwing validation errors.

### Parameter loading (`include/farol2_utils/parameter_utils.hpp`, `src/parameter_utils.cpp`)

Numeric parameter loading is implemented once in the utils library:

| Operation | Vector | Vector3 | Square matrix |
| --- | --- | --- | --- |
| Declare required parameter | `declareVectorParameter` | `declareVector3Parameter` | `declareMatrixParameter` |
| Read existing parameter | `getVectorParameter` | `getVector3Parameter` | `getMatrixParameter` |
| Decode `rclcpp::Parameter` | `vectorParameter` | `vector3Parameter` | `matrixParameter` |

Declaration/get helpers take `rclcpp::Node&` and the parameter name; vectors also
require their expected length and matrices require their dimension. Matrix input
accepts a diagonal vector or a full row-major array. All paths require double
arrays with finite values and valid lengths. Integer/string arrays are separate
ROS parameter types and are not silently converted. Matrix loading checks the
representation, not symmetry or positive definiteness; apply `validation.hpp`
checks for those model-specific requirements.

```cpp
auto cg = farol2_utils::declareVector3Parameter(node, "vehicle.center_of_gravity");
auto inertia = farol2_utils::declareMatrixParameter(node, "vehicle.inertia_tensor", 3);
auto gains = farol2_utils::getVectorParameter(node, "controller.gains", 4);
```

Use `get*` for already-declared parameters, including automatically declared
parameter overrides. The declaration helpers deliberately do not suppress ROS
redeclaration errors. Direct decode functions are useful when a parameter object
is already available, and can be tested without creating ROS nodes.

Link `farol2_utils::farol2_utils` (or use the existing ament dependency). These
functions use ROS parameter types; the Eigen-only geometry/validation target and
simulator physics library remain independent of this ROS loading code.

### 2) Filters

#### Low-pass filter (`include/farol2_utils/filters/low_pass_filter.hpp`, `src/low_pass_filter.cpp`)
State-space low-pass filter with configurable order and design/discretization method.

Key points:
- designs: Butterworth, Bessel
- discretization methods: Euler, ZOH, Tustin
- supports derivative outputs (`y`, `dy`, `ddy`)
- optional angle-aware wrapping flow

Detailed documentation: `docs/low_pass_filter.md`

#### Notch filter (`include/farol2_utils/filters/notch_filter.hpp`, `src/notch_filter.cpp`)
Causal 2nd-order IIR notch filter for online rejection around one target frequency.

Key points:
- runtime sample-by-sample update
- runtime Nyquist guard (`f0 < fs/2`)
- explicit internal state reset
- low computational cost per sample

Detailed documentation: `docs/notch_filter.md`

## Build and dependency notes
- library target links Eigen3 (required by low-pass state-space implementation)
- package includes unit tests for notch filter behavior and parameter validation
- error model follows C++ exceptions (`std::invalid_argument`, `std::runtime_error`)

## Quick usage examples

### LowPassFilter
```cpp
#include <farol2_utils/filters/low_pass_filter.hpp>

farol2_utils::LowPassFilter lpf;
lpf.configure(2.0, 0.1, 2, "butterworth", "tustin", false);

for (double u : signal) {
  lpf.step(u, 0.1);
  const double y = lpf.y();
  const double dy = lpf.dy();
  (void)y;
  (void)dy;
}
```

### NotchFilter
```cpp
#include <farol2_utils/filters/notch_filter.hpp>

farol2_utils::NotchFilter nf;
nf.configure(10.0, 5.0);  // f0 [Hz], Q

for (double x : signal) {
  nf.step(x, 0.01);       // dt [s]
  const double y = nf.y();
  (void)y;
}
```
