# FAROL2 Scripts

This folder contains shell helpers for common FAROL2 development and ROS 2
simulation tasks.

## Usage

Add the loader to your `~/.bashrc`:

```bash
echo 'source ${COLCON_ROOT}/src/farol2/farol2_scripts/source_all.sh' >> ~/.bashrc
```

## Migrating to `farol`

The pure Bash CLI at `farol2_cli/` provides a modern command interface while
these scripts remain available for backwards compatibility:

```bash
src/farol2/farol2_cli/install.sh

farol build
farol clean
farol bag crop my_bag 10 20 -a
farol bridge serial /dev/ttyACM0
farol kill ros
```

See `farol2_cli/README.md` for the complete command mapping and usage.

## Scripts

### `display_git_branch_in_prompt.sh`

Adds the current Git branch to the Bash prompt and changes the prompt color for
some known hostnames.

### `farol_build.sh`

Defines `farol_build`, which builds the workspace at `COLCON_ROOT` with
`colcon build --symlink-install`. It automatically chooses a parallel worker
count while leaving one or two CPU cores free.

```bash
farol_build
```

### `farol_clean.sh`

Defines `farol_clean`, which searches inside `COLCON_ROOT` for `build`,
`install`, and `log` directories and removes them after confirmation. It has
safety checks for dangerous paths.

```bash
farol_clean
```

### `crop_bag.sh`

Defines `crop_bag`, which crops a ROS 2 bag between two times relative to the
bag start. It can crop selected topics, wildcard topic patterns, or all topics.

```bash
crop_bag my_bag 10 20 /imu/data /gps/fix
crop_bag my_bag 10 20 '/magicelectric0/measurement/*'
crop_bag -a my_bag 10 20
crop_bag -o my_crop my_bag 10 20 /imu/data
```

### `serial_bridge.sh`

Defines `serial_bridge`, which creates a virtual serial port bridged to a real
serial device using `socat`.

```bash
serial_bridge /dev/ttyACM0 /tmp/pico 115200
```

### `nuke_ros2.sh`

Defines `nuke_ros2`, which looks for ROS 2-related processes, shows what it
found, then stops them with escalating signals. It protects known tools such as
PlotJuggler, Foxglove Bridge, and bag recording processes, and stops the ROS 2
daemon at the end.

```bash
nuke_ros2
```
