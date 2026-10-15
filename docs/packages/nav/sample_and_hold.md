# sample_and_hold

## Purpose

`sample_and_hold` is the first stage of the `farol2_nav` pipeline (implemented by `SampleAndHoldFilter`).

Its role is to map raw measurements into the shared navigation state with minimal processing, so downstream filters can focus only on refinement.

## What it does

- Copies GNSS latitude/longitude to state.
- Converts valid lat/lon to UTM (GeographicLib) and fills northing/easting/zone.
- Accepts direct `ned_utm` updates (overrides UTM position channels when present).
- Copies velocity over ground (NED) and velocity through water (body).
- Copies depth and altimeter.
- Converts IMU quaternion to roll/pitch/yaw and IMU angular velocity to deg/s.

## Why it matters in the pipeline

- It provides a deterministic baseline state every tick.
- It centralizes sensor-to-state mapping in one place.
- It allows other filters to read state values instead of duplicating sensor parsing.

In practice, this acts like a sample-and-hold stage: each tick uses the latest available measurements and holds values when no fresh measurement arrives.

## Inputs used

- `gnss`
- `utm_ned`
- `velocity_over_ground`
- `velocity_through_water`
- `depth`
- `altimeter`
- `imu`

## Outputs affected in shared state

- Position: latitude, longitude, northing, easting, UTM zone
- Velocities: over-ground NED, through-water body
- Vertical channels: depth, altimeter
- Attitude and angular velocity

## Notes

- `measurements` controls which sensor topics `filter_node` subscribes to.
- `plugins.sample_and_hold.initializer_measurements` controls which measurements must arrive before this filter is considered initialized. When omitted, it defaults to `measurements`.
- Freshness gating is done by `filter_node` before filter execution (timeouts), not inside this filter.
- It does not estimate hidden states; it only maps measurements.
