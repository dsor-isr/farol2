# farol2_inner_loop

## Overview
`farol2_inner_loop` provides the inner-loop PI/PID controllers for the vehicle degrees of freedom and an `open_loop` node. It replaces the old ROS 1 `pid` package: the executable is `inner_loop_node` (ROS node name `inner_loop`), the message is `farol2_inner_loop/msg/PidDebug` and the service `farol2_inner_loop/srv/ChangeParams`.

Implemented channels: `surge`, `sway`, `heave`, `depth`, `altitude`, `yaw`, `pitch`, `roll`, `yaw_rate`, `pitch_rate`, `roll_rate`.

Nodes:
- [inner_loop_node](inner_loop_node.md): the controllers. Reads references and the navigation state, publishes a body wrench.
- `open_loop` (see below): maps a surge reference directly to rpm (magicelectric).

## Runtime model
- A single generic reference callback handles all channels. Runtime wiring is a `ControllerConfig` table (state accessor, reference accessor, optional rate accessor, output accumulator, debug filler).
- Only channels with `enabled: true` in the YAML are instantiated, subscribed and executed.
- Each timer tick: validate `dt`, run the active controllers, add each output to the wrench of its axis, publish the six axes, reset the wrench.

```text
inner_loop.yaml -> loadParams -> active controllers
active controllers -> createControllers + initialiseSubscribers
timerCallback -> callControllers -> executeController
executeController -> body_wrench_request_ accumulation
body_wrench_request_ -> allocation/thrust_{x,y,z}, torque_{x,y,z} -> reset
```

## Units and routing
- Linear references/states: m/s and m. Angular references, states and rates: **degrees and deg/s** at the interface, converted with `deg2rad` internally.
- Output mapping (several channels on one axis **add up**):

| Channel | Output |
|---|---|
| surge | `thrust_x` |
| sway | `thrust_y` |
| heave, depth | `thrust_z` |
| altitude | `thrust_z`, sign inverted to NED (down-positive) |
| yaw, yaw_rate | `torque_z` |
| pitch, pitch_rate | `torque_y` |
| roll, roll_rate | `torque_x` |

- Yaw can use the course angle instead of heading when `course_instead_of_yaw` is true (parameter, or the service, see [inner_loop_node](inner_loop_node.md)).

## Safety and gating
- A cycle is skipped if `dt <= 0` or `dt > 2/node_frequency`.
- A channel only runs if at least one `NavigationState` has been received and its reference is newer than `ref_timeout` (default 2 s). After that the reference generator resets to the current state and nothing is published for the channel. **Publish references continuously (at least 5 Hz).**
- A channel not enabled in the YAML ignores its reference topic (no subscriber exists).

## open_loop node
`inner_loop/open_loop` (executable `open_loop`, namespace `/<ns>/inner_loop`) listens to `inner_loop/ref/surge` and publishes `allocation/rpm_command` directly: `rpm = ref * surge.gain` (parameters `surge.enabled`, default false, and `surge.gain`, default 1.0, in the `open_loop:` block of the vehicle's `inner_loop.yaml`; magicelectric uses `gain: 2200.0`). It runs at a fixed 10 Hz with a fixed 0.2 s reference timeout, publishes two identical rpm entries, and outputs zero rpm while `mission_status` is 0. It does not clamp to the rpm limits of `allocation.yaml`. Enabled with the `open_loop` launch argument (`start_farol2.launch.py` starts it together with `pid: true`; it only acts when `surge.enabled: true`, which is the case for magicelectric and not for delfim).

## Testing helper
`test/ref_pub.py` publishes test references (hard-coded to `/magicelectric0/`; edit it for other vehicles). For manual tests use `ros2 topic pub -r 5 /<ns>/inner_loop/ref/surge std_msgs/msg/Float32 "{data: 1.0}"` (see [docs/runbook.md](../../docs/runbook.md)).
