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