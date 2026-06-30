# ROS2 Motion Planning Package with CasADi

A ROS2 motion planning package that leverages [CasADi](https://web.casadi.org/) for optimal control and trajectory planning. This package provides tools for planning smooth, efficient trajectories for autonomous systems using nonlinear optimization.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Prerequisites](#prerequisites)
- [CasADi Installation](#casadi-installation)
- [Package Installation](#package-installation)

## Requirements

- **ROS2**: Jazzy or later (or other supported distribution)
- **Python**: 3.10 or later (if using CasADi with Python bindings)
- **C++**: C++17 or later
- **CMake**: 3.16 or later
- **System Libraries**: IPOPT solver for optimization

## Installation

### Prerequisites

Ensure you have ROS2 and basic build tools installed:

```bash
# Update system packages
sudo apt update
sudo apt upgrade

# Install ROS2 build essentials (if not already installed)
sudo apt install python3-colcon-common-extensions build-essential cmake
```

### CasADi Installation

**Note:** CasADi is compiled from source with IPOPT support. This may take 10-15 minutes.

```bash
# Update and install IPOPT dependencies
sudo apt update
sudo apt install coinor-libipopt-dev

# Clone CasADi repository
git clone https://github.com/casadi/casadi.git
cd casadi

# Create and configure build directory
mkdir build
cd build

cmake .. \
  -DWITH_PYTHON=OFF \
  -DWITH_IPOPT=ON \
  -DCMAKE_BUILD_TYPE=Release

# Build and install (using all available cores)
make -j$(nproc)
sudo make install
sudo ldconfig

# Verify installation
ls /usr/local/lib | grep casadi
```

**Expected output:** You should see `libcasadi.so` and related library files.

If you need Python bindings (optional), rebuild with `-DWITH_PYTHON=ON`:

```bash
# In the build directory
cmake .. -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### Package Installation

This motion planning node is used in combination with the **farol2 bezier console**, a web-based interface for interactive mission planning and visualization. For setup and usage instructions, see the [farol2 bezier console documentation](https://github.com/uuooffjjkkaa-new/farol2_bezier_console/blob/main/README.md).