# Time Simulation Package

This package provides a time acceleration feature and a checker node. It sets the simulation time by publishing it on /clock, it is configurable with the sim_time.speedup paramater in the ros.yaml file. If use_sim_time is false this package may run but the other nodes will not be affected by it and run at wall time.

## Nodes

**sim_clock**
-Calculates the wall frequency at which it must publish and with that it sets the sim time. 

**clock_checker**
- Calculates the error of the publishing of the clock and if it's above 50% prints a warning.

## Overview

`SimClock` manages simulation time by:
- Reading configuration parameters for real frequency and speedup factor
- Publishing clock messages at a wall-clock rate adjusted by the speedup factor
- Accumulating simulation time independently of real time

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sim_time.node_frequency` | double | 10.0 | Simulation update frequency (in sim time) in Hz |
| `sim_time.speedup_factor` | double | 1.0 | Time acceleration factor (>1 = faster, <1 = slower) |

Both parameters must be greater than 0.
node_frequency must equal the largest frequency on the code since timers and publishers firing are dependant on the /clock topic

## Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Reliable, Transient Local | Simulated time messages |

## Implementation Details

- Uses wall-clock timer (ignores ROS time), wall_frequency = real_frequency*speedup_factor
- Computes wall period from simulation timestep divided by speedup factor
- Publishes clock messages at calculated intervals

## To Have in mind...

- By speeding up the program a lot dont forget to also speed up the publishing of references, otherwise the pid node wont receive any references during several iterations
- Don't forget that by speeding up the launching of the nodes by being spaced by a few decimals of a second that means several seconds in sim time, so give it a few seconds before any interactions
- To check how well the PC is doing, do the following command "ros2 topic hz /clock" this will give you the frequency of publishments in that topic, expect speedup*node_frequency, let it stabilize for a while to have a better computation

