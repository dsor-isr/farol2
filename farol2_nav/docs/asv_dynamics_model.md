# asv_dynamics_model

## Purpose
`asv_dynamics_model` (class `AsvDynamicsModelFilter`) is a model-only stage for single-propeller + rudder surface vehicles (magicelectric). It integrates a surge / sway / yaw-rate model driven by the **measured thruster rpm and rudder angle** and writes the modelled through-water velocity into the shared state, so that later filters (`current_estimator`, `yaw_rate_ekf`) have a velocity-through-water estimate when no sensor provides one.

## Inputs
- `measurement/thruster_rpm` (`farol2_interfaces/ThrusterRPM`): only `rpm[0]` is used; clamped to `rpm_min..rpm_max` and rate-limited by `rpm_rate_limit` (rpm/s).
- `measurement/control_surface_angle` (`ControlSurfaceAngle`): `angle[0]` in degrees (rudder), limited to `control_surface_angle_limit`.

Both measurements must be listed in the node's `measurements` parameter.

## Model
Thrust: `tau_u = 2 rho rps^2 D^4 K_T (1 - u / (rps * pitch))` (sign follows the rpm). Rudder torque from a lift/drag model with `K_L`, `K_D0`, `K_D1` and the arm `rudder_arm`. State `(u, v, r)` is integrated with explicit Euler each tick:

```text
u_dot = ( tau_u + m_v v r + x_u u + x_uu |u| u ) / m_u
v_dot = ( -m_u u r + Y_v v + Y_vv |v| v ) / m_v
r_dot = ( tau_r + bias + m_uv u v - d1 r - d2 r|r| ) / m_r
```

## Outputs
- State: `velocity_through_water_body = (u, v, 0)`. (The angular velocity write is commented out in the code.)
- Debug topics (relative to the node, not remapped): `asv_dynamics_model/uvr` (`geometry_msgs/Vector3`: u, v, r in m/s and rad/s) and `asv_dynamics_model/tau` (x: surge force, y: rudder torque, z: yaw rate in deg/s).

## Parameters (`plugins.asv_dynamics_model.*`, all optional with defaults)
`rpm_min` -2000, `rpm_max` 2000, `rpm_rate_limit` 200, `rho` 1025, `prop_pitch` 0.381, `prop_diameter` 0.4318, `k_t_bp` 0.061461915, `m_u` 3100, `m_uv` -2.967651, `m_v` 3102.967651, `x_u` 0, `x_uu` -96.2270, `Y_v` -500, `Y_vv` -1500, `m_r` 88.47884, `damping` 14.334052, `damping_quadratic` 260.675031, `rudder_arm` 4.0, `K_L` 1.398093, `K_D0` 0.000575, `K_D1` 0.520834, `torque_bias` 0, `init_u`, `init_v`, `init_r` 0, and the rudder angle limit.

## Usage
Add `"asv_dynamics_model"` to `filters` **before** the filters that consume its output, for example `["asv_dynamics_model", "current_estimator", "yaw_rate_ekf"]` (see `farol2_nav/config/nav.yaml`).
