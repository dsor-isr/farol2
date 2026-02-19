# SimClock Node

A ROS 2 node that publishes simulated time to the `/clock` topic, enabling time-accelerated or time-decelerated simulation.

## Overview

`SimClock` manages simulation time by:
- Reading configuration parameters for real frequency and speedup factor
- Publishing clock messages at a wall-clock rate adjusted by the speedup factor
- Accumulating simulation time independently of real time

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sim_time.real_frequency` | double | 50.0 | Simulation update frequency (in sim time) in Hz |
| `sim_time.speedup_factor` | double | 1.0 | Time acceleration factor (>1 = faster, <1 = slower) |

Both parameters must be greater than 0.

## Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Reliable, Transient Local | Simulated time messages |

## Implementation Details

- Uses wall-clock timer (ignores ROS time), wall_frequency = real_frequency*speedup_factor
- Computes wall period from simulation timestep divided by speedup factor
- Publishes clock messages at calculated intervals

