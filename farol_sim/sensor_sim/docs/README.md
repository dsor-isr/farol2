# sensor_sim

## Description
The `sensor_sim` package is designed to read the actual simulation state and produce measurement outputs that mimic the data a sensor on a real vehicle would generate. Users have the option to include or exclude noise in the measurements based on their preferences.

## Nodes
- **`sim_measurements`**: This node processes the true simulation values and generates measurement messages for various sensors, including GNSS, depth sensors, and IMUs. It has the capability to introduce Gaussian noise to the outputs. Users can control the activation of these sensors through the corresponding YAML configuration file.
