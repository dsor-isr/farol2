# farol2_interfaces

Shared messages and services. Package-specific interfaces live in their own packages: `farol2_planning` (`PathData`, spawn services), `farol2_path_following` (`StartPF`, `SetPF`, `PFDebug`, ...), `farol2_waypoint` (`SendWpType1`), `farol2_inner_loop` (`PidDebug`, `ChangeParams`) and `farol2_nav` (debug and tune interfaces).

The repository's `docs/README.md` still lists `ProcessState` and `Measurement`, which no longer exist.

## NavigationState

The output of `farol2_nav` and the input of every controller. Published on `/<ns>/nav/filter/state`.

| Field | Type | Unit / frame | Filled by |
|---|---|---|---|
| `header` | `std_msgs/Header` | | nav |
| `global_position` | `GCS` (`latitude`, `longitude`) | deg | GNSS |
| `utm_position` | `UTM` (`northing`, `easting`, `utm_zone`, `northp`) | m | GNSS or `ned_utm` |
| `depth` | float32 | m, positive down | depth sensor |
| `altimeter` | float32 | m above the bottom | altimeter |
| `altitude_wgs84` | float32 | m, ellipsoidal | not filled (always 0) |
| `altitude_local_datum` | `LocalDatum` (`altitude_local_datum_reference`, `altitude`) | m | not filled (always 0) |
| `velocity_over_ground` | `geometry_msgs/Vector3` | m/s, ground-relative, body axes | DVL bottom track or GNSS ([GAPS #4](../../GAPS.md#4-gnss-velocity-ned-is-treated-as-a-body-frame-velocity-high)) |
| `speed_over_ground` | float64 | m/s | nav (norm of `velocity_over_ground`) |
| `course_over_ground` | float32 | deg | not computed ([GAPS #14](../../GAPS.md#14-course_over_ground-is-never-computed-medium)) |
| `velocity_through_water` | `geometry_msgs/Vector3` | m/s, water-relative, body axes | DVL water track or `asv_dynamics_model` |
| `speed_through_water` | float64 | m/s | nav (norm of `velocity_through_water`) |
| `attitude` | `Attitude` (`roll`, `pitch`, `yaw`) | deg; yaw in [0, 360), 0 = north, clockwise | IMU |
| `angular_velocity` | `geometry_msgs/Vector3` (`p`, `q`, `r`) | deg/s | IMU |
| `current_velocity_ned` | `geometry_msgs/Vector3` | m/s, NED | `current_estimator` |
| `current_speed`, `current_direction` | float64 | m/s; deg, 0 = north, clockwise | `current_estimator` |
| `thruster_rpm` | float64[] | RPM | `measurement/thruster_rpm` |
| `control_surface_angle` | float64[] | deg | `measurement/control_surface_angle` |

## Other messages

| Message | Fields | Used for |
|---|---|---|
| `ThrusterRPM` | `header`, `float64[] rpm` | `allocation/rpm_command` (order = `allocation.thrusters.frames`) and `measurement/thruster_rpm` |
| `ControlSurfaceAngle` | `header`, `float64[] angle` (deg) | `allocation/control_surface_angle` and `measurement/control_surface_angle` |
| `Depth` | `header`, `depth`, `depth_variance` | `measurement/depth` |
| `StateConsole` | position (`x` east, `y` north, `z` down), velocities, attitude and rates (deg), `gps_good`, `imu_good`, `battery_level`, `altitude`, `status` bit field (`STATUS_*`) | Console vehicle state (`/<ns>/State`) and planning's virtual target |
| `Section`, `MultiSection`, `Formation` | Mission sections and formation offsets | Published by `console_parser`; no subscribers in the stack |
| `Attitude`, `GCS`, `UTM`, `LocalDatum` | | Parts of `NavigationState` |
| `Thruster` | `header`, `uint8[] id`, `float64[] value` | Not used in `farol2`; subscribed by the Seabotix drivers in `farol2_drivers` ([GAPS #39](../../GAPS.md#39-medusa-thruster-drivers-cannot-receive-rpm_command-high)) |

## Services

| Service | Request → response | Status |
|---|---|---|
| `ManageProcess` | `uint8 action`, `string name` → `int32 status`, `string message` | Left over from the ROS 1 process manager; unused ([GAPS #25](../../GAPS.md#25-dead-and-half-used-interfaces-low)) |

## Conventions

- Angles on topics are in **degrees**; controllers convert to radians internally.
- Positions are absolute UTM, `x` = north and `y` = east inside the GNC stack, but `x` = east and `y` = north in console messages (ENU).
- Body frame is x forward, y starboard, z down.
