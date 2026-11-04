# inner_loop node

Executable `inner_loop_node`, ROS node name `inner_loop`, namespace `/<vehicle_name><vehicle_id>`. Remaps are in `launch/farol2_inner_loop.launch.py`.

## Subscribers
| Topic | Type |
|---|---|
| `nav/filter/state` | `farol2_interfaces/NavigationState` |
| `inner_loop/ref/{surge,sway,heave,depth,altitude,yaw,pitch,roll,yaw_rate,pitch_rate,roll_rate}` | `std_msgs/Float32` (one subscriber per **enabled** channel) |

## Publishers
| Topic | Type |
|---|---|
| `allocation/thrust_x`, `thrust_y`, `thrust_z` | `std_msgs/Float32` (N) |
| `allocation/torque_x`, `torque_y`, `torque_z` | `std_msgs/Float32` (N.m) |
| `inner_loop/pid/<channel>/debug` | `farol2_inner_loop/PidDebug`, only for channels with `debug: true` |

(The `pid` in the debug topic name is historical; the package is `farol2_inner_loop`.)

## Services
| Service | Type | Notes |
|---|---|---|
| `inner_loop/change_params` | `farol2_inner_loop/srv/ChangeParams` | Fields `controller, kp, ki, kd, lpf_wc, tau_min, tau_max, w0, xi, mr`. **Only changes the yaw controller**, whatever `controller` says. |
| `inner_loop/course_control` | `std_srvs/srv/SetBool` | Yaw from course angle (true) or heading (false). The node's internal name is `course_instead_of_yaw`; the launch file remaps it to `/<ns>/inner_loop/course_control`. |

## Parameters
`farol2_bringup/config/<vehicle>/default/inner_loop.yaml`, under `/**: inner_loop: ros__parameters:`.

Global:
| Parameter | Default | Purpose |
|---|---|---|
| `node_frequency` | 10.0 | Loop rate (Hz). |
| `ref_timeout` | 2.0 | Seconds after which a reference is considered stale. |
| `course_instead_of_yaw` | false | Yaw state source: course angle or heading. |
| `lpf_order`, `lpf_method`, `lpf_design` | 2/3, `tustin`, `bessel` | Low-pass filter for the controllers. |
| `controllers` (optional) | | Allow-list of channels considered before `enabled` filtering. |
| `use_heading_rate_as_yaw_rate` | | Present in several YAMLs but **not read by the code**. |

Per channel (`surge`, `sway`, `heave`, `depth`, `altitude`, `yaw`, `pitch`, `roll`, `yaw_rate`, `pitch_rate`, `roll_rate`):
- `enabled` (bool), `debug` (bool, optional).
- PI channels: `kp`, `ki`, `lpf_wc` (rad/s), `tau_min`, `tau_max`.
- PID channels: `kp`, `ki`, `kd`, `lpf_wc`, `tau_min`, `tau_max`.
- Yaw additionally: `delta_implementation`, `use_ref_lpf`, `use_rate_limiter`, `rate_limit` (deg/s), `kffv_lin`, `kffv_sq`, `kffa`.

## Gating
See the "Safety and gating" section of the [README](README.md).
