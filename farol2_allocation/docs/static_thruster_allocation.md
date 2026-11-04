# Static thruster allocation (`allocation.type: thrust`)

Class `StaticThrusterAllocator`. For vehicles with fixed thrusters (delfim: port and starboard).

1. `ThrusterGeometry` reads the TF transform from `allocation.thrusters.base_frame` to every frame in `allocation.thrusters.frames` and builds the 6 x N thrust configuration matrix `T` (force direction = thruster-frame x axis, moment = position x force).
2. The six requested wrench components (`thrust_x..torque_z`) are stacked into `tau`.
3. Thruster forces: `f = T^+ tau`, with `T^+ = T^T (T T^T)^-1` (pseudo-inverse; [theory](old_README.md)).
4. `ThrusterRpmConverter` turns each force into rpm and clamps to `[min_rpm, max_rpm]` ([rpm_conversion](rpm_conversion.md)).
5. The vector is published on `allocation/rpm_command` (`ThrusterRPM`), one entry per frame in the configured order.

Example (delfim, `allocation.yaml`): two frames `thruster_portside_link`, `thruster_starboard_link`; a 1.0 m/s surge reference gave `thrust_x` about 21.8 N and an rpm of about 2543 on both, a 5 deg/s yaw-rate reference gave `torque_z` about 1.34 N.m and rpm `[450.8, -450.8]`.

Parameters: see the table in the [README](README.md); the `rpm_conversion` block for this mode only needs `coef_fwd`, `coef_bwd`, `max_rpm`, `min_rpm`.
