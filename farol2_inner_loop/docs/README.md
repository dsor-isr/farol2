# PID Package

## Overview
The `pid` package provides inner-loop PI/PID control for vehicle DOFs and can now run as a standalone package with its own launch and default configuration.

Implemented channels:
- surge
- sway
- heave
- yaw
- pitch
- roll
- yaw_rate
- pitch_rate
- roll_rate

## Refactored Runtime Model
The node uses a data-driven runtime model with behavior equivalent to the previous implementation:

- A single generic reference callback handles all channels.
- A single generic controller execution function dispatches by controller type.
- Runtime wiring is stored in a `ControllerConfig` table (state accessor, reference accessor, optional rate accessor, output accumulator, debug filler).
- Only enabled controllers are instantiated, subscribed, and executed.

## Architecture (Short)
Runtime control flow is:

1. Load parameters from YAML.
2. Select active channels: optional `controllers` allow-list, then `enabled: true` filter.
3. Create only the active controller objects (PI/PID as required by channel).
4. Create reference subscriptions only for active channels.
5. On each timer tick, validate `dt`, execute active controllers, route outputs to axis publishers, then reset the accumulated wrench.

Compact flow:

```text
pid.yaml -> loadParams -> active controllers
active controllers -> createControllers + initialiseSubscribers
timerCallback -> callControllers -> executeController
executeController -> body_wrench_request_ accumulation
body_wrench_request_ -> thrust/torque publishers -> resetBodyWrenchRequest
```

Key architectural points:
- Channel behavior is explicit by controller type; runtime activation is data-driven.
- Yaw keeps special state selection (`course_angle` vs `orientation.z`).
- Existing unit conversions, gating, and output mapping are preserved.

## Configuration
Default parameters live in `pid/config/pid.yaml`.

### Controller Activation
Controllers are activated through each channel's `enabled` flag. The optional top-level `controllers` allow-list is supported but not required.

Activation flow:
1. Start from optional `controllers` allow-list (if present).
2. Parse controller parameter blocks.
3. Keep only channels with `enabled: true`.

This means memory/runtime cost scales with enabled channels only.

### Yaw Special Logic
Yaw control keeps the original behavior:
- if `course_control == true`, yaw state uses `course_angle`
- otherwise yaw state uses `orientation.z`

### Units and Routing
The node preserves the original unit handling and output routing:

- linear references/states: direct units
- angular references/states/rates: converted with `deg2rad` where previously applied

Output mapping:
- surge -> force.x
- sway -> force.y
- heave -> force.z
- depth -> force.z
- altitude -> force.z (inverted to NED/down-positive force)
- yaw/yaw_rate -> torque.z
- pitch/pitch_rate -> torque.y
- roll/roll_rate -> torque.x

## Safety and Gating
Control execution and publication still use the original gates:

- timer `dt` sanity guard (`dt <= 0` or `dt > 2/f` skips cycle)
- per-controller enable gate
- recent-reference gate (`< 2/f` since last reference)

## Standalone Usage
Build package:

```bash
colcon build --packages-select pid
```

Run with package launch (recommended):

```bash
source install/setup.bash
ros2 launch pid pid.launch.py
```

`ros2 run pid pid_control` requires parameters to be provided (for topic/service names and controller gains).

## Live Parameter Service
The `ChangeParams` service keeps current behavior and updates yaw gains at runtime.

## Node Documentation
- [pid](pid.md)
