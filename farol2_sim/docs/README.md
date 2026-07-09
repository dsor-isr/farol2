# Vehicle Simulator Package

## Description

This package provides simulators for DSOR vehicles. The simulators receive actuator commands and output realistic simulated states to the sensor interfaces. Key features include hydrodynamic current modeling and vehicle-specific allocation simulation.

## Nodes

**auv_sim**
- Receives thruster RPM commands and outputs the corresponding simulated vehicle state
- Designed for AUVs but compatible with any thruster-based maritime vehicle

**magic_electric_sim**
- Receives thruster RPM commands and rudder control inputs (commands or reference setpoints)
- Optionally simulates rudder behavior based on user configuration in the YAML file
- C++ implementation of the Simulink-based simulator

**sim_clock**
- Publishes the simulation `/clock`
- Launched separately from the simulator nodes because it must run with `use_sim_time: false`
