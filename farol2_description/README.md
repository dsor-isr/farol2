# FAROL2 Description Package

## Overview

This package contains the robot description files for the FAROL2 platform. It includes URDF models, mesh files, and configuration parameters that define the robot's structure and properties.

## Dependencies

Install the required XACRO package:

```bash
sudo apt-get install -y ros-jazzy-xacro
```

## Contents

- **urdf/**: URDF model definitions
- **meshes/**: 3D mesh files for visualization and collision
- **config/**: Configuration files for the robot description

## Usage

Load the robot description in your launch files or nodes using the standard ROS 2 description loading mechanisms.

## References

For more information about robot descriptions in ROS 2, see the [official documentation](https://docs.ros.org/en/jazzy/).