# [waypoint](README.md)/SendWpType1

Service used to send a waypoint to the [waypoint](waypoint.md) node. Served on `/<ns>/waypoint/send_wp_standard`, `send_wp_loose` and `send_wp_heading`.

```
float32 x      # UTM northing (m)
float32 y      # UTM easting (m)
float32 yaw    # final yaw (deg), only used by send_wp_heading
---
bool success
string message
```

Special `(x, y)` values: `(-1,-1)` hold current position, `(-3,-3)` waypoint `delta_t` seconds ahead along the heading, `(-2,-2)` stop (returns `success=False`, `message='Stop signal sent'`).
