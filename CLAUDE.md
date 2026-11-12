# FAROL 2

## What this is
ROS 2 Guidance-Navigation-Control stack for DSOR-ISR marine vehicles (Medusa AUVs `mred`/`mblack`/`myellow`, `bluerov`, `delfim` catamaran, `magicelectric` rudder ASV). One set of nodes per vehicle under namespace `/<vehicle_name><id>`; runs either against the built-in simulator (`farol2_sim`) or on real vehicles with drivers from a separate repo (`farol2_drivers_bringup`, not in this tree). Known defects and risks are tracked in `ANALYSIS.md`.

## Tech stack
- ROS 2 Jazzy, Ubuntu 24.04, C++17 (`rclcpp`), Python 3.12 (launch files, `console_server.py`, launch helpers)
- All 12 packages are `ament_cmake` (bringup/sim also install a Python module via `ament_cmake_python`)
- Eigen3 (written against 3.4), GeographicLib, tf2/tf2_ros/tf2_eigen, xacro + robot_state_publisher, Boost (console only)
- RMW: default (Fast DDS); no DDS XML config in repo. No `ros2_control`, Nav2, lifecycle or composable nodes
- Git submodule `3rd_party/nmea_msgs` (no package here depends on it)

## Workspace setup
```bash
# repo is cloned (or symlinked) into <ws>/src/farol2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy   # + apt: libgeographiclib-dev ros-jazzy-xacro
colcon build --symlink-install
source install/setup.bash
```
On the `hpc-dsor` host there is no `/opt/ros`; keep build trees on `/scratch/$USER/` (never on `/home`). The usual setup is `/scratch/$USER/farol2_analysis_ws` (`src/farol2` symlinks to the repo): `source env.sh && source install/setup.bash`. That env has Eigen 5, which needs `CPLUS_INCLUDE_PATH=<env>/include/eigen3` for `farol2_allocation` (already in `env.sh`).

### `/scratch/$USER/` is volatile and node-local
`/scratch` is a local disk of each node and can be wiped without notice. The ROS env, the workspaces and their helper scripts (`env.sh`) may not exist on the node you are on. Check before relying on them, and rebuild instead of treating a missing path as an error. Never keep the only copy of anything there (sources, results, bags, notes go under `/home`).
```bash
ls /scratch/$USER/farol/manual/mamba/envs/jazzy/bin/colcon /scratch/$USER/farol2_analysis_ws/install/setup.bash
```
Rebuild (needs internet; skip the steps whose output still exists):
```bash
# 1. micromamba + ROS 2 Jazzy env (RoboStack), about 500 packages
M=/scratch/$USER/farol/manual; mkdir -p $M/mm && cd $M
curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C mm bin/micromamba
export MAMBA_ROOT_PREFIX=$M/mamba
mm/bin/micromamba create -y -n jazzy -c conda-forge -c robostack-jazzy python=3.12 \
  ros-jazzy-ros-base colcon-common-extensions compilers cmake pkg-config ninja \
  ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime \
  ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-sensor-msgs ros-jazzy-rosgraph-msgs ros-jazzy-std-srvs \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-ros2launch \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-eigen3-cmake-module eigen geographiclib geographiclib-cpp libboost-devel

# 2. workspace + env.sh
W=/scratch/$USER/farol2_analysis_ws; mkdir -p $W/src && cd $W
ln -sfn ~/farol2 src/farol2
cat > env.sh <<'EOF'
E=/scratch/$USER/farol/manual/mamba/envs/jazzy
export PATH=$E/bin:$PATH
export CONDA_PREFIX=$E
for f in $E/etc/conda/activate.d/*.sh; do source $f; done
export CPLUS_INCLUDE_PATH=$E/include/eigen3${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}
EOF

# 3. build (12 packages, a few minutes)
source env.sh && colcon build --symlink-install && source install/setup.bash
```

## Commands
```bash
colcon build --packages-select farol2_nav            # one package
colcon build --packages-up-to farol2_inner_loop      # package + deps
colcon test --packages-select farol2_utils farol2_sim && colcon test-result --verbose
colcon test --packages-select farol2_utils --ctest-args -R test_notch_filter   # single test
./build/farol2_sim/test_dynamics --gtest_filter='*Name*'                      # single gtest case
farol build | farol build sim | farol clean | farol source   # optional CLI, after farol2_cli/install.sh
```
- Functional tests exist only in `farol2_utils` (gtest) and `farol2_sim` (gtest + `test/test_simulation_launch.py`).
- `colcon test` on any other package only runs `ament_lint_auto` linters (flake8/uncrustify/lint_cmake/pep257/xmllint), which currently fail everywhere; treat those as noise unless asked to fix style.
- Currently failing functional test: `farol2_utils` `NotchFilterTest.RejectsInvalidParametersAndUsage`.
- No CI, no formatter config, no pre-commit hooks in the repo.

## Sim vs real
**Never launch real-hardware configurations. Agents only build, run unit tests, and edit code.** (Simulation launches are fine.)
```bash
# sim: one world (owns /clock) per DDS domain, then vehicles join it
ros2 launch farol2_bringup start_simple_world.launch.py ros_domain_id:=41
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=myellow vehicle_id:=0 ros_domain_id:=41
# real (do not run): ros2 launch farol2_bringup start_vehicle_real.launch.py vehicle_name:=...
```
- Mode is chosen by which top-level launch is used: `start_vehicle_sim` = `farol2_sim` + `start_farol2.launch.py` with `use_sim_time:=true`; `start_vehicle_real` = `start_farol2.launch.py` with `use_sim_time:=false` (driver include is present but commented out).
- Sim-only: `farol2_sim/`, `start_simple_world.launch.py`, `start_vehicle_sim.launch.py`, `config/<vehicle>/<profile>/sim.yaml`, `config/simple_world.yaml`. Real-only: `start_vehicle_real.launch.py`. Everything else is shared.
- Sim launches set `ROS_DOMAIN_ID` from the `ros_domain_id` arg (default 0), ignoring the shell value; CLI tools must `export ROS_DOMAIN_ID=<same>`.
- `vehicle_id` empty = auto-pick next free id from the ROS graph.

## Architecture
```
farol2_bringup     launch entry points + ALL parameter YAMLs: config/<vehicle>/<profile>/{nav,inner_loop,allocation,waypoint,path_following,planning,sim,addons}.yaml
farol2_description xacro per vehicle + tf_static.launch.py (robot_state_publisher)
farol2_interfaces  shared msgs (NavigationState, ThrusterRPM, ControlSurfaceAngle, Depth, ...)
farol2_utils       angles.hpp (deg/rad, wrap), geometry, LPF/MAF/notch filters, parameter + validation helpers, StaticTransformLookup
farol2_sim         sim_clock + vehicle_sim (6-DOF dynamics, thrusters, rudder, sensors)
farol2_nav         filter_node: sample_and_hold + optional plugin pipeline (src/filters/)
farol2_planning    path_node: path sections (line/arc/circle/bernoulli) + speed profiles, via services
farol2_path_following  path_following_node: 11 algorithms in src/algorithms/, switched via services
farol2_waypoint    waypoint_node: wp_standard / wp_loose / wp_heading
farol2_inner_loop  inner_loop_node (per-axis PID + reference generator) and open_loop (surge ref -> RPM)
farol2_allocation  allocation_node: body wrench -> thruster RPM (TF-derived pseudo-inverse) or rudder angle
farol2_console     console_parser (mission strings) + console_server.py (HTTP UI); not included by start_farol2
farol2_cli         bash `farol` helper CLI (build/clean/bag/drivers/kill)
```
Data flow: sensors (`measurement/*`) -> nav filter -> `nav/filter/state` -> {planning + path_following | waypoint} -> `inner_loop/ref/*` -> inner_loop -> `allocation/{thrust,torque}_*` -> allocation -> `allocation/rpm_command`, `allocation/control_surface_angle` -> sim or drivers.

## Node graph summary
All names below are under `/<vehicle><id>/`. Loop rate is `node_frequency` (10 Hz in all shipped configs; sim 100 Hz).

| Node (launch name) | Subscribes | Publishes | Services |
|---|---|---|---|
| `sim/vehicle_sim` | `allocation/rpm_command` (ThrusterRPM), `allocation/control_surface_angle` (ControlSurfaceAngle) | `measurement/{imu (Imu), gnss (NavSatFix), velocity_over_ground, velocity_through_water (Vector3Stamped), depth (Depth), altimeter (Range), thruster_rpm, control_surface_angle}`, `sim/*` truth (Vector3), TF `world`->`base_link`, `joint_states` | - |
| `navigation_filter` | `measurement/*` listed in param `measurements` | `nav/filter/state` (NavigationState), `nav/<plugin>/state` | `current_estimator/tune`, yaw-rate EKF tune |
| `planning/paths` | `path_following/gamma` (Float32), `nav/filter/state` | `path_following/path_data` (PathData), `path_following/virtual_state` | `ResetPath`, `SetMode`, `SpawnLinePath`, `SpawnArc2DPath`, `SpawnCircle2DPath`, `SpawnBernoulliPath`, `SetConstVdRabbit`, `SetConstVdVehicle` |
| `path_following` | `nav/filter/state`, `path_following/path_data`, `path_following/vc`, `mission_status` (Int8) | `inner_loop/ref/{surge,sway,yaw,yaw_rate}` (Float32), `path_following/gamma`, `path_following/debug`, `mission_status` | `path_following/{Start,Stop,UpdateGains,ResetVT,Set<Algo>}` |
| `waypoint` | `nav/filter/state`, `mission_status`, `turn_radius_flag` | `inner_loop/ref/{surge,sway,yaw,yaw_rate}`, `mission_status` | `waypoint/send_wp_{standard,loose,heading}` (SendWpType1) |
| `inner_loop` | `nav/filter/state`, `inner_loop/ref/<axis>` (Float32) | `allocation/{thrust_x,y,z,torque_x,y,z}` (Float32), `inner_loop/pid/<axis>/debug` | `inner_loop/change_params`, `course_instead_of_yaw` |
| `inner_loop/open_loop` | `inner_loop/ref/surge`, `mission_status` | `allocation/rpm_command` | - |
| `actuator_allocation` | `allocation/{thrust,torque}_*`, `nav/filter/state`, `mission_status` | `allocation/rpm_command` (ThrusterRPM), `allocation/control_surface_angle` | - |
`mission_status` values: 0 idle, 4 waypoint, 6 path following. `/world/sim_clock` publishes `/clock`.

## Frames
- `world` is **NED** (x north, y east, z down); `<ns>/base_link` is body **FRD**. This is the marine convention, not REP-103 ENU/FLU; do not "fix" it locally.
- TF: `world -> <ns>/base_link` dynamic, broadcast only by `vehicle_sim`. `<ns>/base_link -> <ns>/{imu,gnss,dvl,depth,altimeter}_link` and `-> <ns>/thruster_*_link` static, from the vehicle xacro via `robot_state_publisher` (`frame_prefix=<ns>/`).
- Thruster force acts along the thruster link +x axis; allocation and sim both derive geometry from TF, so the xacro is the single source of actuator/sensor placement.
- Positions are UTM northing/easting (m). `NavigationState` attitude is in **degrees** and angular velocity in **deg/s**; `inner_loop/ref/{yaw,pitch,roll}` and rate refs are in degrees; `PathData.tangent` is in radians.

## Conventions
- Node layout: constructor calls `loadParams()`, `initialiseSubscribers()`, `initialisePublishers()`, `initialiseServices()`, `initialiseTimer(s)()`; one node class per executable, `rclcpp::spin` on a single-threaded executor; no callback groups.
- Topic/service names in C++ are short relative constants (`static constexpr char TOPIC_SUB_*[]`); the full names are assigned only by `remappings` in each package's launch file. A new topic needs both.
- Parameters: read once at startup, mostly `declare_parameter<T>("name")` with no default (missing YAML key = startup exception). `inner_loop`, `path_following`, `console_parser` instead use `automatically_declare_parameters_from_overrides(true)` + `get_parameter`. Vector/matrix params go through `farol2_utils::declareVectorParameter` etc. No runtime parameter callbacks; live tuning is done with custom services.
- YAML files start with `/**:` and are keyed by the node name given in the launch file `name=`.
- QoS: everything is default reliable/volatile keep-last; depth 1 for commands, references and state consumers, depth 10 in the nav filter. `/clock` is published reliable + transient-local. No sensor-data (best-effort) profiles are used.
- Time: use the node clock (`now()`, `get_clock()`, `create_timer`) so sim time works; `create_wall_timer` is used only for TF-availability retries. Durations/`dt` are computed from node-clock stamps.
- Angles: convert with `farol2_utils::deg2rad/rad2deg`, wrap with `wrapToPi/wrapTo2Pi` (`farol2_utils/angles.hpp`).
- Validation helpers (`farol2_utils/validation.hpp`: `requirePositive`, `requireFinite`, ...) throw `std::invalid_argument`; used heavily in `farol2_sim`.
- Tests are gtest files in `<pkg>/test/test_*.cpp` registered with `ament_add_gtest`.
- Git: branch from `dev` (`feature/*`, `fix/*`, `experiment/*`), never commit to `main` (field-validated code only).

## Gotchas
- The launch `name=` overrides the C++ node name; a YAML keyed by the wrong name is silently ignored and the node runs on in-code defaults. Check with `ros2 param get` after changing either.
- Only `farol2_sim` and `farol2_path_following` honour the `config_to_use` launch arg; the other launch files load `config/<vehicle>/default/` unconditionally.
- Unqualified `abs(x)` on a double resolves to the integer overload in several translation units here; always write `std::abs`.
- The Jazzy conda env on the HPC host ships Eigen 5, where `ament_target_dependencies(<tgt> Eigen3)` adds no include path; link `Eigen3::Eigen` instead.
- Sim time starts near 0 and jumps when the first `/clock` arrives; nodes log one "dt value abnormal" warning at startup. `ros2 node list` via the daemon can be stale; use `--no-daemon`.
- `farol kill ros` (`farol2_cli/libexec/farol-kill-ros`) kills every process of the user matching broad patterns such as `ros2` or `/install/.*/lib/`; do not use it on shared machines.
- Vehicle sim refuses to start without an advancing `/clock` in the same domain (start the world first); a second world in the same domain is rejected.
- `farol2_console` writes mission files relative to its working directory (`path_folder`) and opens an HTTP port (`PORT`, 7080).
- `docs/` folders contain large PNGs; skip them when grepping.

## Do / Don't
- Do add any new parameter to every `farol2_bringup/config/<vehicle>/default/<file>.yaml` that the node loads, since most parameters have no default.
- Do add a remap in the package launch file for every new topic/service, following the `/<ns>/<area>/<name>` pattern.
- Do keep units explicit in names/comments (deg vs rad, RPM, N, N.m) and convert at node boundaries.
- Do add gtest coverage for new pure-logic code (controllers, filters, allocation math) alongside the node.
- Do run a sim launch on a private `ros_domain_id` (not 0) with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` when verifying behaviour on shared hosts.
- Don't launch `start_vehicle_real.launch.py` or anything from a drivers package.
- Don't change frame conventions, message units or `mission_status` codes without updating every consumer listed in the node graph.
- Don't put build/install/log directories under `/home` on the HPC host, and don't commit them (`.gitignore` covers `build/ install/ log/`).
- Don't assume anything under `/scratch/$USER/` exists or survives: it is per-node and volatile (see Workspace setup for the rebuild).
- Don't copy the raw `new`/`delete` controller ownership pattern from `farol2_waypoint`/`farol2_path_following`/`farol2_planning` into new code; use `std::unique_ptr`.
