# Thruster + rudder allocation (`allocation.type: thrust_rudder`)

Class `RudderAllocator`. For vehicles with one common-mode propeller and a rudder (magicelectric).

Inputs: the six wrench components, `nav/filter/state` and `mission_status`.

- **Gate:** while `mission_status == 0` the node outputs nothing (zero rpm, no rudder command).
- **Thruster:** only the requested surge force `thrust_x` is used. It is converted to rpm by `ThrusterRpmConverter`, taking the through-water surge velocity into account in propeller-model mode ([rpm_conversion](rpm_conversion.md)). The result is published on `allocation/rpm_command`, one entry per thruster frame. With `allocation.thrusters.open_loop: true` this output is suppressed and the `open_loop` node publishes rpm instead.
- **Rudder:** the requested `torque_z` gives `delta = torque_z / (K_s * cm_distance * 1.75)` (the `1.75` is hard-coded in `rudder_allocator.cpp`), saturated to `allocation.rudder.limits` and published in degrees on `allocation/control_surface_angle`.

Implementation caveats:
- `K_L`, `K_D0`, `K_D1` are read and the lift/drag and flow-angle quantities are computed, but they are not applied to the output (`rudder_x_body_drag` is stored and never added to the surge force; `solveDeltaFromTau` is never called). `NavigationState.attitude.yaw` and `angular_velocity.z` are in degrees but are used as radians inside `RudderAllocator::compute`; this only affects the unused terms today.
- The old description in [old_README.md](old_README.md) divides by `||V_s||^2`; the code does not.

Example (magicelectric): `allocation.yaml` has `K_s: 1.0`, `K_L = K_D0 = K_D1 = 0`, `cm_distance: 4.0`, rudder limits +-36 deg, one frame `thruster_middle_link`, `rpm_conversion.mode: 0`, rpm limits +-2200.
