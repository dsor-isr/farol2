# Waypoint Package

Waypoint outer loop for the vehicle. Given a target position it computes the references (surge, sway, yaw, yaw rate) that make the vehicle converge to it, and publishes them to the inner loop. The package also owns part of the `mission_status` protocol (below).

## Nodes
* [waypoint](waypoint.md)

## Services
* [sendWpType1](sendWpType1.md)

## Dependencies
* `farol2_interfaces` (`NavigationState`)
* Eigen3

## The `mission_status` topic

`mission_status` (`std_msgs/Int8`) was called `Flag` in the ROS 1 FAROL. It tells every outer-loop node which one is in control, and the allocation / open-loop nodes use it as an enable. Default code values:

| mission_status | State |
| --- | --- |
| 0 | Idle |
| 4 | Waypoint |
| 6 | Path following |

It can be extended with custom states. The topic is both published and subscribed by `waypoint` and `path_following`.

## Quick use

```bash
ros2 service call /delfim0/waypoint/send_wp_standard farol2_waypoint/srv/SendWpType1 "{x: <northing>, y: <easting>, yaw: 0.0}"
```

See [docs/runbook.md](../../docs/runbook.md) for a complete example and the stop command.
