# position_current_ekf

## Purpose

`position_current_ekf` estimates horizontal current velocity while smoothing horizontal position.

It is designed to work after `sample_and_hold`: it consumes the already-populated position/yaw channels from shared state, then writes back corrected position and current estimate.

## State and model

The filter state is:

- $x = [x_{pos}, y_{pos}, c_x, c_y]^T$

with:

- $x_{pos}, y_{pos}$: horizontal position (northing/easting)
- $c_x, c_y$: water-current velocity components in NED

Prediction model:

$$
\dot{x}_{pos} = v_{x,model} + c_x
$$
$$
\dot{y}_{pos} = v_{y,model} + c_y
$$

where $(v_{x,model}, v_{y,model})$ is obtained from:

1. an RPM-to-body-velocity model that estimates both surge and sway,
2. projection to NED using current yaw.

The body-frame velocity model is:

$$
\dot{u} = \frac{1}{m_u}(\tau_u + X_u u + X_{uu}|u|u)
$$

$$
\dot{v} = \frac{1}{m_v}(-m_u u r + Y_v v + Y_{vv}|v|v)
$$

with $r$ taken from the current yaw-rate estimate.

Discrete covariance propagation uses Jacobian $F$ and process noise $Q_d = Q \cdot dt$.

Measurement model (when position measurement exists in current tick):

$$
z = Hx + v, \quad H =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}
$$

## Input dependencies

- Position measurements from shared state (`northing`, `easting`) populated by `sample_and_hold`.
- Yaw from shared attitude state.
- Yaw rate from shared angular-velocity state.
- RPM command (`rpm_command`) for surge model.

## Outputs written to shared state

- `current_velocity_ned.{x,y}` from EKF current estimate.
- `velocity_through_water_ned.{x,y}` from model projection.
- `velocity_through_water_body.{x,y}` from modeled surge and sway.
- Optional overwrite of position:
  - `northing`, `easting` if `override_position_state=true`.

## Practical behavior

- EKF initializes on first available position measurement.
- If no position measurement is available in a tick, prediction still runs (open-loop update).
- RPM model includes:
  - command clamp,
  - rate limiting.

## Parameters

All parameters are under `plugins.position_current_ekf.*`.

Noise and covariance:

- `process_noise_pos`
- `process_noise_current`
- `measurement_noise_pos`
- `init_cov_pos`
- `init_cov_current`
- `init_current_x`
- `init_current_y`
- `override_position_state`

Surge model and propulsion constants:

- `rpm_min`, `rpm_max`, `rpm_rate_limit`
- `rho`, `prop_pitch`, `prop_diameter`, `k_t_bp`
- `m_u`, `m_uv`, `m_v`
- `x_u`, `x_uu`
- `Y_v`, `Y_vv`

## Runtime tuning service

Service: `position_current_ekf/tune` (`farol2_nav/srv/TunePositionEkf`)

Request fields:

- `process_noise_pos`
- `process_noise_current`
- `measurement_noise_pos`

All values must be positive.

## Assumptions and limitations

- Current is modeled in horizontal plane only.
- Performance depends strongly on propulsion model calibration.
