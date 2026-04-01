# pid Node

## Diagram
<!-- ![pid Diagram](img/pid.png) -->

## Subscribers
- `topics.subscribers.nav_state` (`farol_interfaces/msg/NavigationState`)
- `topics.subscribers.<controller>_ref` (`std_msgs/msg/Float32`) for each enabled controller

Notes:
- Reference subscriptions are created dynamically for enabled channels only.
- Angular references are converted to radians internally.

## Publishers
- `topics.publishers.thrust_x` (`std_msgs/msg/Float32`)
- `topics.publishers.thrust_y` (`std_msgs/msg/Float32`)
- `topics.publishers.thrust_z` (`std_msgs/msg/Float32`)
- `topics.publishers.torque_x` (`std_msgs/msg/Float32`)
- `topics.publishers.torque_y` (`std_msgs/msg/Float32`)
- `topics.publishers.torque_z` (`std_msgs/msg/Float32`)
- `topics.publishers.debug.<controller>` (`pid/msg/PidDebug`) for controllers with `debug: true`

Output routing:
- surge -> `thrust_x`
- sway -> `thrust_y`
- heave -> `thrust_z`
- yaw/yaw_rate -> `torque_z`
- pitch/pitch_rate -> `torque_y`
- roll/roll_rate -> `torque_x`

## Services
- `topics.services.change_params` (`pid/srv/ChangeParams`)
- `topics.services.course_control` (`std_srvs/srv/SetBool`)

Notes:
- `change_params` currently updates yaw gains.
- `course_control` toggles yaw state source: course angle vs heading.

## Parameters
- `node_frequency` (double): control loop frequency in Hz.
- `course_control` (bool): if true yaw uses `course_angle`, else `orientation.z`.
- `lpf_order`, `lpf_method`, `lpf_design`: low-pass filter settings passed to PID controllers.

Per-controller block (for each of `surge`, `sway`, `heave`, `yaw`, `pitch`, `roll`, `yaw_rate`, `pitch_rate`, `roll_rate`):
- `enabled` (bool): activates controller creation/execution.
- `debug` (bool, optional): enables debug publisher for that controller.
- gains/limits depending on controller type:
	- PI: `kp`, `ki`, `lpf_wc`, `tau_min`, `tau_max`
	- PID: `kp`, `ki`, `kd`, `lpf_wc`, `tau_min`, `tau_max`
	- yaw PID also requires: `kffv_lin`, `kffv_sq`, `kffa`

Optional:
- `controllers` (string list): allow-list of controllers to consider before `enabled` filtering.

Gating behavior:
- Controller executes only if enabled and reference is recent (`< 2/f`).
- Timer cycle is skipped if `dt` is invalid (`dt <= 0` or `dt > 2/f`).
