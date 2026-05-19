# LowPassFilter

## Goal
`LowPassFilter` smooths noisy references/signals and provides filtered derivatives for controllers.

## Location
- Header: `include/farol2_utils/filters/low_pass_filter.hpp`
- Implementation: `src/low_pass_filter.cpp`

## Public API
```cpp
class LowPassFilter {
public:
	LowPassFilter();
	// Legacy signature
	void configure(double wc, double Ts, int order=2,
								 std::string design="butterworth",
								 std::string method="tustin",
								 bool wrap_angle=false,
								 bool use_fixed_Ts=false);
	// Optional Ts signature
	void configure(double wc, int order=2,
								 std::string design="butterworth",
								 std::string method="tustin",
								 bool wrap_angle=false,
								 bool use_fixed_Ts=false,
								 double Ts=0.0);
	void reset(double u_hat0 = 0.0, double du_hat0 = 0.0);
	void step(double u);
	void step(double u, double dt);
	double y() const;
	double dy() const;
	double ddy() const;
	bool isConfigured() const;
};
```

## Parameter meaning
- `wc`: cutoff in rad/s
- `Ts`: nominal/fixed sampling period (required only when `use_fixed_Ts=true`)
- `order`: filter order (`>= 1`)
- `design`: continuous-time pole placement style (`"butterworth"` or `"bessel"`)
- `method`: discretization (`"tustin"`, `"zoh"`, fallback Euler)
- `wrap_angle`: enables angular unwrapping/wrapping path during update
- `use_fixed_Ts`:
	- `false`: variable-step mode, `step(u, dt)` uses runtime `dt`
	- `true`: fixed-step mode, filter always uses configured `Ts` and ignores `dt`

Step options:
- `step(u, dt)`: always available (uses fixed `Ts` when fixed mode is active)
- `step(u)`: convenience overload for fixed-step mode only (`use_fixed_Ts=true`)

## Validation and error model
- `configure`: throws `std::invalid_argument` when:
	- `wc <= 0`
	- `Ts <= 0`
	- `order < 1`
- `step`: throws `std::runtime_error` if called before `configure`

## Theory summary

### Continuous-time model
Filter is represented in state-space form:

$$
\dot{x}(t) = A x(t) + B u(t), \quad y(t) = x_0(t)
$$

State vector uses derivative-chain form:

$$
x = [y, \dot{y}, \ddot{y}, \ldots]^T
$$

### Pole design
- Butterworth: maximally flat magnitude in passband.
- Bessel: improved phase/group-delay behavior.

Both designs build denominator polynomial then map to companion-form `A` and `B`.

### Discretization
At each `step`, code updates discrete model with current `dt`:
At each `step`, code updates discrete model with:

- `dt_used = dt` in variable-step mode
- `dt_used = Ts` in fixed-step mode

- Euler:
$$
A_d = I + A\,dt, \quad B_d = B\,dt
$$

- ZOH (matrix exponential of augmented system):
$$
\Phi = \exp\left(\begin{bmatrix}A & B\\0 & 0\end{bmatrix} dt\right)
$$

- Tustin (bilinear transform):
$$
(I - \tfrac{dt}{2}A) A_d = I + \tfrac{dt}{2}A,\quad
(I - \tfrac{dt}{2}A) B_d = dt\,B
$$

## Runtime update flow
1. Optional angle unwrapping when `wrap_angle=true`
1. Save previous state/history (`last_x_`, `last_dy_`, `last_dt_`)
1. Recompute `A_d`, `B_d` from selected method and current `dt`
1. State propagation:
	 $$x_k = A_d x_{k-1} + B_d u_k$$
1. Optional wrap output back to `[0, 2*pi)` if unwrapping occurred

## Derivative outputs
- `y()` returns `x(0)`
- `dy()`:
	- direct state `x(1)` for order >= 2
	- finite difference fallback for order 1
- `ddy()`:
	- direct state `x(2)` for order >= 3
	- finite-difference fallback otherwise

## Reset behavior
`reset(u_hat0, du_hat0)`:
- zeroes/reallocates internal state vectors
- initializes `x(0)=u_hat0`
- initializes `x(1)=du_hat0` when available
- clears derivative history baseline

## Practical guidance
- Use `method="tustin"` as default robust choice for most control loops.
- Use Bessel when phase lag is more critical than roll-off sharpness.
- Keep `dt` consistent for predictable digital response.
- Enable `wrap_angle` for heading-like periodic states.
