# farol2_interfaces

## Description
`farol2_interfaces` holds the custom ROS 2 messages and services shared by the FAROL 2 stack. (Package-specific messages live in their own package: `PidDebug` in `farol2_inner_loop`, `PathData` in `farol2_planning`, `PFDebug` in `farol2_path_following`, `SendWpType1` in `farol2_waypoint`, `CurrentEstimatorDebug` / `YawRateEkfDebug` and the tune services in `farol2_nav`.)

## Messages
| Message | Description |
| --- | --- |
| `NavigationState` | Output of the navigation filter: position (GCS and UTM), depth, altimeter, altitude, velocities over ground / through water (body frame), attitude (deg), angular velocity (deg/s), course over ground, current estimate, actuator states. See the field table below. |
| `UTM` | Part of `NavigationState`: northing, easting (m), zone, hemisphere. |
| `GCS` | Part of `NavigationState`: latitude, longitude (deg). |
| `LocalDatum` | Part of `NavigationState`: a local reference for the altitude. |
| `Attitude` | roll, pitch, yaw (deg). |
| `Depth` | `Header` + `depth` + `depth_variance`. Measurement published by the depth sensor / sim to `measurement/depth`. |
| `ControlSurfaceAngle` | `Header` + `float64[] angle` (degrees, one per surface, e.g. rudder). |
| `ThrusterRPM` | `Header` + `float64[] rpm`. Allocation output, sim input (`allocation/rpm_command`). |
| `Thruster` | `Header`, `uint8[] id`, `float64[] value`. Only used with the Medusa thrusters (marked TODO for removal). |
| `StateConsole` | State message for the web console (ENU: x east, y north). |
| `Formation`, `MultiSection`, `Section` | Console mission types (formation and multi-section missions). |

## Services
| Service | Description |
| --- | --- |
| `ManageProcess` | Left over from the ROS 1 bringup process manager. Not used by any node in this repository. |

`ProcessState` and `Measurement` (documented in older versions of this page) no longer exist.

## NavigationState fields
| Field | Type | Unit |
|---|---|---|
| `global_position` (`GCS`) | latitude, longitude | deg |
| `utm_position` (`UTM`) | northing, easting, zone, northp | m |
| `depth` | float32 | m |
| `altimeter` | float32 | m |
| `altitude_wgs84`, `altitude_local_datum` | | m |
| `velocity_over_ground` | Vector3, NED velocity expressed in the body frame | m/s |
| `speed_over_ground`, `course_over_ground` | | m/s, deg |
| `velocity_through_water`, `speed_through_water` | Vector3 in body frame | m/s |
| `attitude` | roll, pitch, yaw | deg |
| `angular_velocity` | p, q, r | deg/s |
| `current_velocity_ned`, `current_speed`, `current_direction` | estimated current; direction 0 = north, clockwise | m/s, deg |
| `thruster_rpm[]`, `control_surface_angle[]` | actuator states | rpm, deg |
