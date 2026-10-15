# farol2_inner_loop

Per-axis inner-loop control. Two nodes:

- `inner_loop` (executable `inner_loop_node`): PID controllers that turn references into body forces and torques.
- `open_loop`: a surge-to-RPM shortcut for vehicles without a closed surge loop (written for magicelectric).

## inner_loop

### Controllers

Eleven channels exist. Each one is created only if its YAML block has `enabled: true` (and, if the optional `controllers` list is given, it appears there).

| Channel | State used (from `nav/filter/state`) | Rate used | Output | Reference unit |
|---|---|---|---|---|
| `surge` | `velocity_through_water.x` | numerical derivative | `thrust_x` | m/s |
| `sway` | `velocity_through_water.y` | numerical derivative | `thrust_y` | m/s |
| `heave` | `velocity_through_water.z` | numerical derivative | `thrust_z` | m/s |
| `depth` | `depth` | `velocity_over_ground.z` | `thrust_z` | m |
| `altitude` | `altimeter` | `-velocity_over_ground.z` | `-thrust_z` | m |
| `yaw` | `attitude.yaw`, or `course_over_ground` when `course_instead_of_yaw` | `angular_velocity.z` | `torque_z` | deg |
| `pitch` | `attitude.pitch` | `angular_velocity.y` | `torque_y` | deg |
| `roll` | `attitude.roll` | `angular_velocity.x` | `torque_x` | deg |
| `yaw_rate` | `angular_velocity.z` | numerical derivative | `torque_z` | deg/s |
| `pitch_rate` | `angular_velocity.y` | numerical derivative | `torque_y` | deg/s |
| `roll_rate` | `angular_velocity.x` | numerical derivative | `torque_x` | deg/s |

Angular references and states are converted to radians before control. Channels sharing an output (for example `yaw` and `yaw_rate`) are summed.

### Control cycle

On each tick at `node_frequency`:

1. Skip the cycle if `dt <= 0` or `dt > 2 / node_frequency`.
2. For each enabled channel that has a navigation state and has received at least one reference:
   - If the last reference is older than `ref_timeout`, reset the reference generator to the current state and output nothing for that channel.
   - Otherwise run the **reference generator** (optional rate limiter, then optional low-pass filter of order `lpf_order`, design `lpf_design`, discretization `lpf_method`, cut-off `lpf_wc`), which also provides `dref` and `ddref`.
   - Run the **PID** on the generated reference.
3. Publish each axis that has a fresh reference, then zero the accumulators.

### PID law

`ControllerPID` (`src/controller_pid.cpp`) has two forms, selected per channel with `delta_implementation` (default **true** when the key is absent).

- **Delta (velocity) form**: `tau_dot = -ki·e - kp·ė - kd·ė_rate + kffv_lin·v̇ + kffv_sq·v̇|v̇| + kffa·dddref`, integrated with back-calculation anti-windup (`Ka = 1/dt`) and saturated to `[tau_min, tau_max]`. See [GAPS #34](../../GAPS.md#34-delta-form-pid-quadratic-feed-forward-medium) for the quadratic term.
- **Standard form**: `tau = -kp·e + I - kd·(rate - dref)`, where `I` integrates `-ki·e` with back-calculation and only while `|tau| < tau_max/2`.

Error is `state - reference` (wrapped to [-π, π] for angles). When no rate is supplied, the state rate is a Tustin-discretized filtered derivative with cut-off `lpf_wc`.

### Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `nav/filter/state` | `farol2_interfaces/NavigationState` |
| sub | `inner_loop/ref/<channel>` | `std_msgs/Float32` (only for enabled channels) |
| pub | `allocation/{thrust,torque}_{x,y,z}` | `std_msgs/Float32` |
| pub | `inner_loop/pid/<channel>/debug` | `farol2_inner_loop/PidDebug` (when `<channel>.debug: true`) |

### Services

| Service | Type | Notes |
|---|---|---|
| `inner_loop/change_params` | `farol2_inner_loop/ChangeParams` | Changes **yaw** gains only, whatever `controller` says. If `w0`, `xi`, `mr` are all > 0, gains come from `kp = mr(w0² + 20ξ²w0²)`, `ki = 10·mr·ξ·w0³`, `kd = 12·mr·ξ·w0`. `tau_min`/`tau_max` > 0 are applied; `lpf_wc` is ignored |
| `course_instead_of_yaw` | `std_srvs/SetBool` | Switch yaw feedback to course over ground. Lives at `/<ns>/course_instead_of_yaw`; the `course_control` remap in the launch file does not apply |

### Parameters

| Parameter | Type | Meaning |
|---|---|---|
| `node_frequency` | double | Control rate (Hz) |
| `ref_timeout` | double, default 2.0 | Seconds without a reference before a channel stops |
| `course_instead_of_yaw` | bool | Initial value of the yaw feedback switch |
| `lpf_order`, `lpf_method`, `lpf_design` | int, string, string | Reference low-pass filter (`euler`/`zoh`/`tustin`, `butterworth`/`bessel`) |
| `use_heading_rate_as_yaw_rate` | bool | In the YAML but not read by the code |
| `controllers` | string[] (optional) | Allow-list of channels |
| `<channel>.enabled` | bool | Create this channel |
| `<channel>.debug` | bool | Publish `PidDebug` |
| `<channel>.kp`, `ki`, `kd` | double | Gains (default 0) |
| `<channel>.kffv_lin`, `kffv_sq`, `kffa` | double | Feed-forward on rate, rate², reference acceleration (default 0) |
| `<channel>.tau_min`, `tau_max` | double | Output saturation |
| `<channel>.delta_implementation` | bool, default true | Delta or standard form |
| `<channel>.use_ref_lpf` (alias `use_lpf`) | bool, default true | Filter the reference |
| `<channel>.lpf_wc` | double (rad/s), default 1.0 | Reference filter and derivative cut-off |
| `<channel>.use_rate_limiter`, `rate_limit` | bool, double | Rate-limit the reference |
| `<channel>.use_filtered_ref_for_control` | bool, default false | Use filtered reference as the PID setpoint |

## open_loop

Publishes `[surge_ref · surge.gain, surge_ref · surge.gain]` to `allocation/rpm_command` at a fixed 10 Hz while a surge reference newer than 0.2 s exists, and `[0, 0]` when `mission_status` is 0. It always assumes two thrusters.

| Direction | Topic | Type |
|---|---|---|
| sub | `inner_loop/ref/surge` | `std_msgs/Float32` |
| sub | `mission_status` | `std_msgs/Int8` |
| pub | `allocation/rpm_command` | `farol2_interfaces/ThrusterRPM` |

Parameters: `surge.enabled` (bool, default false), `surge.gain` (RPM per m/s, default 1.0).

`start_farol2.launch.py` launches it for every vehicle, so it competes with `actuator_allocation` for `rpm_command` wherever `surge.enabled` is true ([GAPS #3](../../GAPS.md#3-two-nodes-publish-to-the-same-rpm_command-topic-high)).

## Test helper

`test/ref_pub.py` publishes test references; it is not installed as an executable.

## Known gaps

[#3](../../GAPS.md#3-two-nodes-publish-to-the-same-rpm_command-topic-high), [#5](../../GAPS.md#5-surge-loop-has-no-feedback-on-most-vehicles-high), [#11](../../GAPS.md#11-course_control-remap-misses-the-real-service-name-medium), [#12](../../GAPS.md#12-change_params-always-edits-the-yaw-controller-medium), [#13](../../GAPS.md#13-use_heading_rate_as_yaw_rate-is-never-read-medium), [#26](../../GAPS.md#26-stale-build-dependency-low), [#34](../../GAPS.md#34-delta-form-pid-quadratic-feed-forward-medium).
