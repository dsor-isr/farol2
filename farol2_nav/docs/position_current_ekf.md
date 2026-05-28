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

1. an RPM-to-surge model,
2. projection to NED using current yaw.

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
- RPM command (`rpm_command`) for surge model.

## Outputs written to shared state

- `current_velocity_ned.{x,y}` from EKF current estimate.
- `velocity_through_water_ned.{x,y}` from model projection.
- `velocity_through_water_body.x` from modeled surge.
- Optional overwrite of position:
  - `northing`, `easting` if `override_position_state=true`.

## Practical behavior

- EKF initializes on first available position measurement.
- If no position measurement is available in a tick, prediction still runs (open-loop update).
- RPM model includes:
  - command clamp,
  - rate limiting,
  - optional fixed-speed override after staying near a target RPM for a configured time.

## Parameters

All parameters are under `plugins.position_current_ekf.*`.

Noise and covariance:

- `process_noise_pos`
- `process_noise_current`
- `measurement_noise_pos`
- `init_cov_pos`
- `init_cov_current`
- `override_position_state`

Surge model and propulsion constants:

- `rpm_min`, `rpm_max`, `rpm_rate_limit`
- `rho`, `prop_pitch`, `prop_diameter`, `k_t_bp`
- `m_u`, `x_u`, `x_uu`

Override behavior:

- `override_velocity`
- `override_rpms`
- `override_timeout_s`

## Runtime tuning service

Service: `position_current_ekf/tune` (`farol2_nav/srv/TunePositionEkf`)

Request fields:

- `process_noise_pos`
- `process_noise_current`
- `measurement_noise_pos`

All values must be positive.

## Assumptions and limitations

- Model currently estimates only surge in body frame (no sway model).
- Current is modeled in horizontal plane only.
- Performance depends strongly on propulsion model calibration.
