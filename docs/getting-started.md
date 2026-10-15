# Getting started

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (`ros-jazzy-desktop` or `ros-jazzy-ros-base`)
- System packages: `libgeographiclib-dev`, `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-tf2-eigen`, `ros-jazzy-tf2-geometry-msgs`, `libboost-dev` (needed by `farol2_console` but not declared in its `package.xml`)

```bash
sudo apt install libgeographiclib-dev libboost-dev \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs
```

## Get the code

```bash
mkdir -p ~/dsor/colcon_ws_sim/src
cd ~/dsor/colcon_ws_sim/src
git clone --recurse-submodules -b dev git@github.com:dsor-isr/farol2.git
```

`--recurse-submodules` pulls `3rd_party/nmea_msgs`. `main` holds field-validated code and `dev` the latest integrated code (see `WORKFLOW.md`); this documentation describes `dev`.

The separate repository `dsor_simulation-ros2` is out of date with both branches and cannot launch the current stack ([GAPS #6](GAPS.md#6-dsor_simulation-ros2-launches-packages-that-no-longer-exist-high)). Use `farol2_bringup` directly until it is updated.

## Shell setup

Either use the `farol` CLI (recommended on `dev`):

```bash
cd ~/dsor/colcon_ws_sim/src/farol2/farol2_cli && ./install.sh
```

or the `.bashrc` block in the top-level `README.md`, which defines `COLCON_HOME`, `COLCON_WS`, `set_colcon_ws_function`, `ROSDATA_DIR` and sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. See [packages/cli](packages/cli/README.md) for the CLI commands.

## Build

```bash
cd ~/dsor/colcon_ws_sim
colcon build --symlink-install       # or: farol build
source install/setup.bash            # or: farol source
```

## Run a vehicle in simulation

```bash
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=mred
```

| Argument | Default | Meaning |
|---|---|---|
| `vehicle_name` | `magicelectric` | One of `bluerov`, `delfim`, `magicelectric`, `mblack`, `mred`, `myellow` |
| `vehicle_id` | empty (next free id) | Namespace suffix; the stack runs under `/<vehicle_name><vehicle_id>` |
| `config_to_use` | `default` | Override folder under `farol2_bringup/config/<vehicle>/` (mostly ignored, [GAPS #7](GAPS.md#7-config_to_use-overrides-are-disabled-in-8-of-9-launch-files-medium)) |
| `farol2_clock` | `true` | Start `sim_clock` |

The console web server listens on `http://127.0.0.1:7080/`.

## Drive it

Send a waypoint (absolute UTM northing/easting in metres; `x = y = -1` holds position, `-2` stops, `-3` holds a point ahead):

```bash
ros2 service call /mred0/waypoint/send_wp_standard farol2_waypoint/srv/SendWpType1 "{x: 4290800.0, y: 491900.0}"
```

Follow a line:

```bash
ros2 service call /mred0/SpawnLinePath farol2_planning/srv/SpawnLine "{start_point: [4290800.0, 491900.0, 0.0], end_point: [4290900.0, 491900.0, 0.0], ref_point: [0.0, 0.0, 0.0]}"
ros2 service call /mred0/SetConstVdVehicle farol2_planning/srv/SetConstSpeed "{speed: 0.5, default_speed: 0.5}"
ros2 service call /mred0/path_following/SetBreivik farol2_path_following/srv/SetPF
ros2 service call /mred0/path_following/Start farol2_path_following/srv/StartPF
```

Path points are absolute UTM coordinates; `Line` stores `ref_point` but does not apply it. Service definitions are listed in [packages/planning](packages/planning/README.md) and [packages/path_following](packages/path_following/README.md). Stop with `path_following/Stop` or by sending a waypoint.

Watch the state:

```bash
ros2 topic echo /mred0/nav/filter/state
```

## Run on a real vehicle

`start_vehicle_real.launch.py` starts the same stack with `use_sim_time:=false`. Its driver include points to `farol2_drivers_bringup` in the separate [`farol2_drivers`](https://github.com/dsor-isr/farol2_drivers) repository, and is commented out of the returned actions, so start the drivers yourself (`ros2 launch farol2_drivers_bringup magicelectric_drivers.launch.py`). Only magicelectric and the towed payload have driver launch files, and on `dev` the magicelectric rudder topics do not match ([GAPS #38](GAPS.md#38-magicelectric-rudder-topics-no-longer-match-the-drivers-high)). The file header says to copy it into your own bringup package.

## Record data

```bash
ros2 launch farol2_bringup bag_record.launch.py vehicle_name:=mred vehicle_id:=0 [compression:=true]
```

Records all topics to `<prefix>_<timestamp>`. `farol bag crop|info|play|topics` help post-process bags.
