# yaw_rate_ekf

## Purpose

`yaw_rate_ekf` estimates a smoother and dynamically consistent yaw rate and yaw angle by combining:

- IMU yaw-rate measurements,
- rudder-angle-driven yaw torque model,
- through-water velocity from upstream state.

It is designed as a focused rotational filter for yaw dynamics, updating yaw-rate and yaw channels.

## State and model

Internal state:

- $x = [r, b, g, \psi]^T$

with:

- $r$: yaw rate (rad/s)
- $b$: torque-bias/disturbance term
- $g$: torque gain scale term
- $\psi$: yaw angle (rad, wrapped to $[-\pi, \pi]$ internally)

Continuous model used in prediction:

$$
\dot{r} = \frac{g\tau_r + b + m_{uv}uv - d_1 r - d_2 r|r|}{I}
$$
$$
\dot{b} = 0
$$
$$
\dot{g} = 0
$$
$$
\dot{\psi} = r
$$

where:

- $I$ is yaw inertia,
- $d_1, d_2$ are linear/quadratic damping,
- $\tau_r$ is a rudder-induced torque proxy.

Measurement model:

$$
z_r = r + v_r
$$
$$
z_{\psi} = \psi + v_{\psi}
$$

## Delay-aware correction strategy

The IMU yaw-rate and IMU orientation yaw are each prefiltered with moving-average windows.

Because moving-average introduces delay, the filter keeps a short history of predicted states. A joint correction with measurement vector $z=[z_r, z_{\psi}]^T$ is applied to the delayed state, then states are re-propagated to the present sample. This aligns EKF correction with delayed measurement timing.

## Input dependencies

- IMU angular velocity (`imu.angular_velocity.z`).
- IMU orientation quaternion (`imu.orientation`) used to extract yaw measurement.
- Rudder angle command (`rudder_angle`).
- Through-water velocity (`velocity_through_water_ned`) from shared state.

## Outputs written/published

Shared state:

- `angular_velocity.z` (deg/s) is replaced by EKF estimate.
- `attitude.yaw` (deg) is replaced by EKF estimate.

Debug topics:

- `torque_bias`
- `yaw_rate_filtered`
- `yaw_rate_ekf/current_yaw_rate`
- `yaw_rate_ekf/delayed_yaw_rate`
- `yaw_rate_ekf/innovation`

## Parameters

All parameters are under `plugins.yaw_rate_ekf.*`.

Noise and covariance:

- `process_noise_yaw_rate`
- `process_noise_bias`
- `process_noise_gain`
- `process_noise_yaw`
- `measurement_noise_yaw_rate`
- `measurement_noise_yaw`
- `init_cov_yaw_rate`
- `init_cov_bias`
- `init_cov_gain`
- `init_cov_yaw`

Dynamics:

- `inertia`
- `damping`
- `damping_quadratic`
- `torque_gain`
- `rudder_arm`

Measurement prefilter:

- `measurement_window_samples` (must be >= 1)
- `yaw_measurement_window_samples` (must be >= 1 and currently matched to `measurement_window_samples`)

## Runtime tuning service

Service: `yaw_rate_ekf/tune` (`farol2_nav/srv/TuneYawRateEkf`)

Request fields:

- `process_noise_yaw_rate`
- `process_noise_bias`
- `process_noise_gain`
- `process_noise_yaw`
- `measurement_noise_yaw_rate`
- `measurement_noise_yaw`

All values must be positive.

## Assumptions and limitations

- Rudder torque model is intentionally simple and should be treated as an approximation.
- Filter writes both yaw-rate and yaw channels to shared state.
- Quality depends on realistic damping/inertia parameters and consistent rudder units.
