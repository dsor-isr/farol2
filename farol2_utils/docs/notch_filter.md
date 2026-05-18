# NotchFilter

## Goal
`NotchFilter` removes a narrow frequency component around one target frequency while preserving most of the rest of the signal.

Typical use:
- remove narrow resonance
- suppress a known oscillation line
- keep real-time online filtering (causal, sample-by-sample)

## Location
- Header: `include/farol2_utils/filters/notch_filter.hpp`
- Implementation: `src/notch_filter.cpp`

## Public API
```cpp
class NotchFilter {
public:
  NotchFilter();
  void configure(double f0, double Q,
                 bool use_fixed_Ts=false,
                 double Ts=0.0);
  void reset();
  void step(double x);
  void step(double x, double dt);
  double y() const;
  bool isConfigured() const;
};
```

## Parameter meaning
- `f0`: notch center frequency [Hz]
- `Q`: quality factor (higher `Q` -> narrower notch)
- `use_fixed_Ts`:
  - `false`: variable-step mode, `step(x, dt)` uses runtime `dt`
  - `true`: fixed-step mode, filter always uses configured `Ts` and ignores `dt`
- `Ts`: fixed sampling period [s] used when `use_fixed_Ts=true`
- `dt`: runtime sampling period [s], with `fs = 1/dt`

## Validation and error model
- `configure` throws `std::invalid_argument` when:
  - `f0 <= 0`
  - `Q <= 0`
  - `use_fixed_Ts=true` and `Ts <= 0`
- `step` throws:
  - `std::runtime_error` if called before `configure`
  - `std::invalid_argument` when `dt <= 0`
  - `std::invalid_argument` when `f0 >= fs/2`

Step options:
- `step(x, dt)`: always available (uses fixed `Ts` when fixed mode is active)
- `step(x)`: convenience overload for fixed-step mode only (`use_fixed_Ts=true`)

## Theory

The filter is a 2nd-order IIR biquad notch:

$$
y[k] = b_0 x[k] + b_1 x[k-1] + b_2 x[k-2] - a_1 y[k-1] - a_2 y[k-2]
$$

with:

$$
\omega_0 = 2\pi\frac{f_0}{f_s}, \quad \alpha = \frac{\sin(\omega_0)}{2Q}
$$

Unnormalized coefficients:

$$
b_0=1,\; b_1=-2\cos(\omega_0),\; b_2=1
$$

$$
a_0=1+\alpha,\; a_1=-2\cos(\omega_0),\; a_2=1-\alpha
$$

Normalized by `a0` in implementation.

## Implementation details

### Internal state
- previous inputs: `x1_`, `x2_`
- previous outputs: `y1_`, `y2_`
- current output cache: `y_`

### Runtime sequence per sample
1. validate configuration and runtime `dt`
1. select `dt_used = use_fixed_Ts ? Ts : dt`
1. compute `fs = 1/dt_used` and Nyquist guard
1. update coefficients for current `fs`
1. compute `yk` from biquad equation
1. shift state history (`x2 <- x1 <- x`, `y2 <- y1 <- yk`)

This is causal and suitable for real-time loops.

## Reset behavior
`reset()` zeros all dynamic memory terms (`x1_`, `x2_`, `y1_`, `y2_`, `y_`).

## Example
```cpp
#include <farol2_utils/filters/notch_filter.hpp>

farol2_utils::NotchFilter nf;
nf.configure(10.0, 5.0);  // notch at 10 Hz, moderate bandwidth

const double dt = 0.01;   // 100 Hz
for (double x : signal) {
  nf.step(x, dt);
  const double y = nf.y();
  (void)y;
}
```

## Tuning notes
- Start with `Q` between `3` and `10`.
- Increase `Q` for more selective rejection.
- Very high `Q` can increase transient ringing sensitivity.
- Ensure `f0` safely below Nyquist for expected `dt` jitter range.
