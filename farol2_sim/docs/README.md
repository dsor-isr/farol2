# Vehicle simulation

`vehicle_sim` runs every supported vehicle using the same ROS-independent
Fossen dynamics library. Model parameters, active DOFs and actuator models come
from `farol2_bringup/config/<vehicle>/default/sim.yaml`.

Use [config/vehicle.yaml](../config/vehicle.yaml) as a commented template for a
fictional vehicle with thrusters, a rudder and all currently supported sensors.
It includes both thruster model options, units, frame requirements and sensor
mode choices. Copy and adapt it for your vehicle; its sample TF frames and
physical coefficients do not describe a real vehicle.

`auv_sim` and `magic_electric_sim` have been replaced. External launchers must
use `vehicle_sim`; the old executable-selection launch arguments were removed.

## Organization

| File | Responsibility |
| --- | --- |
| `include/farol2_sim/state.hpp` | Ground-truth state and environment, without ROS types |
| `src/dynamics.cpp` | Mass, Coriolis, relative-flow damping, restoring forces and reduced solve |
| `src/kinematics.cpp` | State-dependent rigid-body kinematics and water/ground velocity selection |
| `src/integrator.cpp` | Semi-implicit stepping and matrix-exponential attitude integration |
| `farol2_utils/geometry.hpp` | Shared skew, rotation, point-velocity and force-to-wrench helpers |
| `farol2_utils/validation.hpp` | Shared scalar, container, matrix and SO(3) validation |
| `farol2_utils/parameter_utils.cpp` | Shared ROS numeric vector/matrix parameter loading |
| `src/thruster.cpp` | RPM limits, deadzone, continuous-time lag/delay and thrust models |
| `src/rudder.cpp` | Angle limits/rate, local flow, lift and drag |
| `src/vehicle_sim_node.cpp` | ROS parameters, TF loading, commands, scheduling and truth publishing |
| `include/farol2_sim/sensors.hpp` | Sensor interface, read-only step inputs and sensor class declarations |
| `src/sensors/gnss.cpp` | Geographic fix, optional UTM position and ground-velocity outputs |
| `src/sensors/sensor_kinematics.cpp` | ROS-independent sensor mounting, velocity and specific-force calculations |
| `src/sensors/mounted_sensor.cpp` | Fixed sensor transform loading and validation from TF |
| `src/sensors/imu.cpp` | Mounted orientation, angular velocity and accelerometer specific force |
| `src/sensors/dvl.cpp` | Bottom/water tracking and body/NED velocity selection |
| `src/sensors/depth_cell.cpp` | Depth below the world water surface |
| `src/sensors/altimeter.cpp` | Synthetic 2–4 m sinusoidal range for filter testing |
| `src/sensors/thruster_rpm.cpp` | Actual simulated motor RPM feedback in actuator order |
| `src/sensors/control_surface_angle.cpp` | Actual control-surface angle feedback in degrees |
| `src/sensors/noise.cpp` | Per-sensor Gaussian noise sampling and legacy noise parameter loading |

The exported CMake target is `farol2_sim::farol2_sim_dynamics`. It depends only on
Eigen, C++17 and the Eigen-only `farol2_utils::farol2_utils_geometry` target.
Actuators produce body wrenches; the dynamics implementation has
no knowledge of thruster counts, RPM commands, rudders, ROS, TF or sensor topics.

## Frames, velocities and units

- Navigation frame: NED. Body axes: forward, starboard, down.
- `position_ned`: body-origin position relative to the configured geographic origin.
- `rotation_body_to_ned`: rotation matrix mapping body vectors into NED.
- `velocity_through_water_body`: **water-relative** surge/sway/heave and body
  angular velocity, expressed in body axes, ordered `[u,v,w,p,q,r]`.
- `current_ned`: water velocity relative to ground, expressed in NED.
- Kinematics produce ground velocity: `v_ground_ned = R * v_water_body + current_ned`.
  This is also `position_dot`. Current is added exactly once, in kinematics.
- Ground velocity in body axes is `v_ground_body = v_water_body + R^T current_ned`.
- Internal/configuration units: m, s, kg, N, N*m, rad, rad/s and kg/m^3.
  Inertia is in kg*m^2. RPM command units remain RPM.

The initial position is local `[north_m, east_m, down_m]` relative to the world
geographic origin configured in `environment.origin`. Initial attitude is RPY in radians and initial angular velocity is body
`[p,q,r]` in rad/s. `initial_state.velocity_reference` must be `ground` or `water`;
all migrated configurations explicitly use `water` for their initial body velocity.

The existing truth topic names are retained. `body_velocity` now consistently
means water-relative body velocity for all vehicles; `orientation` and
`orientation_rate` retain degrees and degrees/s for compatibility. Acceleration
truth is the derivative of the water-relative body velocity in m/s^2 and rad/s^2.
Quaternion/Euler representations are used only for configuration and ROS output;
attitude state and integration use rotation matrices.

## Dynamics and current

The solver stores and integrates relative velocity `nu_r`:

```text
M = M_RB + M_A
M nu_r_dot = tau - C(M, nu_r) nu_r
  - D_l nu_r - D_q (abs(nu_r) .* nu_r) - g

position_dot = R * nu_r.linear + current_ned
```

Dynamics, damping, propeller advance speed and rudder inflow all use water-relative
velocity. Ambient current is a prescribed advection velocity applied by kinematics;
it is never subtracted from the stored velocity or fed back as vehicle acceleration.
This simulator uses a relative-motion model with no current-acceleration or shear
forces. The ROS wrapper computes a constant horizontal current from the world's
configured speed and angle at startup.

DVL publishing uses the same `linearVelocity` conversion as kinematics:

| DVL mode | Reported quantity | Output topic |
| --- | --- | --- |
| `bottom_track` | Velocity over ground | `velocity_over_ground` |
| `water_track` | Velocity through water | `velocity_through_water` |

`sensor.dvl.output_frame` selects `sensor` (default), compensated `body`, or
`inertial` (NED). Sensor output uses the DVL origin and mounting axes, including
the rotational lever-arm velocity. Body output compensates to the body origin
and axes; inertial output expresses the DVL-origin velocity in NED.
The corresponding message frame is the configured sensor frame, vehicle base
frame, or `world`. A nonrotating vehicle drifting with the water has zero water-track velocity and ground velocity
equal to the current. With zero current, both modes agree in the same frame.

Reference: [Fossen's marine craft model](https://www.fossen.biz/html/marineCraftModel.html).

## Model parameters

- `vehicle.mass`: positive rigid-body mass.
- `vehicle.inertia_tensor`: 3 diagonal entries or 9 row-major entries, about CG.
- `vehicle.center_of_gravity`, `vehicle.center_of_buoyancy`: body-frame offsets.
- `vehicle.added_mass_tensor`: 6 diagonal entries or 36 row-major entries.
- `vehicle.linear_damping_tensor`: 6 diagonal entries or 36 row-major entries.
- `vehicle.quadratic_damping_tensor`: 6 nonnegative diagonal entries. A 36-entry
  matrix is also accepted by the loader but must be diagonal.
- `vehicle.restoring.displaced_volume`: displaced volume in m^3.
- `vehicle.restoring.sphere_radius`: zero for constant volume; positive to retain
  the legacy sphere-based approximation near the surface.
- `vehicle.active_dofs`: unique body velocity indices selected from `[0,1,2,3,4,5]`.

Inertia must be symmetric positive definite. Added mass and damping must be
symmetric positive semidefinite. The total mass matrix is checked as positive
definite. Damping uses **positive resistance coefficients**; the migrated YAML
negates the old signed hydrodynamic damping coefficients.

Gravity/buoyancy forces are rotated into the body frame and include moments from
both CG and CB. This is configurable hydrostatics, with an optional legacy surface
approximation; it does not yet provide a general surface-vessel hull model.

For selected velocity components `nu = S * nu_active`, acceleration is solved as:

```text
(S^T M S) nu_active_dot = S^T rhs
```

This accounts for constraint reactions through the reduced mass solve. It does
not compute a full acceleration and then discard components. Inactive through-water velocity
components must initially be zero and remain zero. These are water-relative body-velocity
constraints; they do not generally lock inertial coordinates for an arbitrarily
oriented body. Underactuation does not imply a constrained DOF.

The integrator updates velocity, then position with the updated velocity and
starting attitude, then attitude with `R_next = R * exp(skew(omega_next * dt))`.
Position uses the sum of rotated through-water velocity and NED current.
It is first-order and preserves rotation orthonormality to numerical roundoff.

## Actuators

Thruster geometry is loaded from the configured TF frames in command order.
The local +X axis is the positive thrust axis. Each actuator maps force to a body
wrench using its rotation and `position x force`. Static geometry is cached once
all required transforms are available; simulation waits until then.

`vehicle.actuators` selects `polynomial` or `propeller`:

- Polynomial thrust uses separate positive/negative RPM curves, each `[a,b,c]`:
  `force = a*rpm^2 + b*abs(rpm) + c`. Zero RPM produces zero force.
- Propeller thrust uses the vehicle's through-water surge as advance speed and
  `rho * n*abs(n) * D^4 * K_T * (1 - advance_speed/(n*pitch))`, scaled by the
  explicit `thrust_scale`. `n` is revolutions/s.
  Thruster position/orientation still determine the applied force and moment;
  they do not modify advance speed. Equal RPM on symmetric propellers therefore
  does not introduce additional yaw damping from rotational inflow.
- `minimum_rpm`, `maximum_rpm`, `deadzone`, `pole` and `delay` configure the motor.
  Zero pole means ideal response. Zero delay is supported, and fractional delays
  are handled by elapsed time instead of an integer number of callbacks.
- `fixed_rpm` and `fixed_rpm_value` are explicit scenario settings.

`vehicle.rudder` enables the separate rudder model, including body-frame position,
zero-angle orientation, joint name, angle limits, rate limit and lift/drag
coefficients. It includes sway force as well as surge drag and yaw moment.

The default angle interface remains `ControlSurfaceAngle` in degrees. Optional
`command_mode: rate` retains the legacy normalized `rudder_cmd` interface:
commands above +0.5 or below -0.5 move at `command_speed` rad/s. Internally all
angles and rates are radians. `rate_limit: 0` gives ideal angle tracking.

Rudder feedback and joint states are published from the actuator's actual angle.
The identified rudder frame is explicitly configured for now; reconciling it with
moving-joint TF is a separate geometry task.

## Migration notes

- All six shipped vehicles now use `vehicle_sim` and explicit model parameters.
- Magic Electric uses `[0,1,5]` (surge, sway, yaw). Other vehicles retain their
  previous six-DOF simulation. Magic Electric's inactive roll/pitch inertias are
  placeholders, not identified physical values.
- Magic Electric's effective surge/sway masses and yaw inertia were mapped into
  the shared matrices. Previously unused sway damping is now applied, and rudder
  sway force and consistent current handling change its trajectory.
- Magic Electric simulates two physical thrusters, ordered port/left then
  starboard/right, each with `thrust_scale: 1.0`. `fixed_rpm: false` makes it use
  the open-loop controller's two RPM commands. Initial velocity is zero; with
  the default zero current and no commands, the vehicle remains stationary.
- Magic Electric allocation and simulation use the same two physical thruster
  frames, in the same order. `thrust_rudder` allocation sends only surge to this
  symmetric pair, giving equal thrust while the rudder handles yaw. Allocation
  retains `open_loop: true`, so the open-loop controller supplies RPM commands.
  The Xacro contains two thrusters and one rudder, without a virtual middle
  thruster or alternate allocation branches. The rudder's 4 m arm and existing
  joint name are retained.
- Existing AUV densities were converted from kg/l to kg/m^3, with explicit
  displaced volume. Fully submerged buoyancy is preserved, but the sphere size
  and near-surface response change to reflect SI dimensions.
- Existing provisional inertia/damping values are retained. These changes unify
  model structure; they do not establish calibrated vehicle fidelity.
- `config_to_use` now overlays the simulator's default configuration.

## Build and verification

From the colcon workspace, rebuild the simulator and its changed bringup configs:

```bash
colcon build --packages-select farol2_utils farol2_sim farol2_bringup
colcon test --packages-select farol2_sim
colcon test-result --verbose
```

The core can also be built and tested without ROS:

```bash
cmake -S farol2_sim -B /tmp/farol2-sim-core \
  -DFAROL2_SIM_CORE_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/farol2-sim-core -j2
ctest --test-dir /tmp/farol2-sim-core --output-on-failure
```

Core tests cover force/mass response, constraint reactions, current handling,
Coriolis structure, damping, restoring moments, SO(3) integration and actuators.
ROS builds additionally parse all six YAML files with ROS's parameter parser and
exercise their dynamics for 20 seconds without requiring DDS sockets.

For the live check, source the rebuilt workspace and use an unused ROS domain:

```bash
ROS_DOMAIN_ID=159 python3 farol2_sim/test/check_vehicle_configs.py \
  /path/to/install/farol2_sim/lib/farol2_sim/vehicle_sim /path/to/farol2
```

This starts all six common simulator nodes with their Xacro descriptions and one
test clock, sends actuator commands, and checks finite motion and Magic Electric's
planar constraints, sway, yaw and rudder feedback. It requires local DDS sockets.

## World clock and DDS domains

Start one world instance before launching its vehicles. The world currently owns
only its clock, named `/world/sim_clock`, and publishes global `/clock` within its
DDS domain. Clock rate, speedup and the environment are configured in
[simple_world.yaml](../../farol2_bringup/config/simple_world.yaml), independently
of vehicle model files. `world_config` selects another world YAML file on both
the world and vehicle launchers.

World and vehicle launches default to `ros_domain_id:=0` when omitted, regardless
of the shell environment. Pass a different ID explicitly for
independent instances, even if they use the same world configuration:

```bash
# Terminal 1: first world.
ros2 launch farol2_bringup start_simple_world.launch.py ros_domain_id:=41

# Terminal 2: vehicle in the first world.
ros2 launch farol2_bringup start_vehicle_sim.launch.py \
  vehicle_name:=magicelectric vehicle_id:=0 ros_domain_id:=41

# Terminal 3: independent copy of the same world.
ros2 launch farol2_bringup start_simple_world.launch.py ros_domain_id:=42

# Terminal 4: same vehicle name and ID in the second world.
ros2 launch farol2_bringup start_vehicle_sim.launch.py \
  vehicle_name:=magicelectric vehicle_id:=0 ros_domain_id:=42
```

The domain is set before ROS probes or child nodes initialize. All ROS nodes in a
vehicle launch inherit it, including ID discovery and robot_state_publisher.
Vehicle launchers default to domain 0. For a nonzero world, pass its
`ros_domain_id` explicitly to each vehicle launch. Other ROS CLI tools, RViz and external nodes must use that same
domain to see the world. Launch independent instances in separate launch processes.
Domains isolate ROS discovery/traffic; they are not a security boundary and do not
isolate HTTP console ports or other non-ROS resources.

The world launcher rejects a domain with an existing `/clock` publisher. The clock
also stops if a competing publisher is discovered after startup. This catches
concurrent world launches after DDS discovery; it is not a cross-process lock.

Both `start_vehicle_sim.launch.py` and standalone `farol2_sim.launch.py` wait for
fresh, advancing `/clock` messages from exactly one publisher in the selected
domain before starting their nodes. `clock_timeout` defaults to 5 wall seconds.
Increase it for very slow clocks or low speedup settings.
An absent/frozen clock or multiple publishers produces a launch error, with the
domain and the command to start its world. A published topic name or one retained
clock sample is insufficient. External simulators can provide `/clock` instead of
the Farol world if they satisfy the same readiness check.

This is a startup check. If time stops later, ROS-time processing stops advancing;
launch does not treat an intentional pause as a failure. Navigation and allocation
processing now use ROS-time timers. Clock generation and TF discovery continue to
use wall timers. Resets, backward jumps and physics catch-up remain separate tasks.
The clock advances a fixed simulation interval per wall callback; speedup is a
requested rate, not guaranteed real-time performance under overload.

The old `sim_clock.launch.py` and `farol2_clock` vehicle argument are removed.
Clock settings were removed from vehicle YAML, including the example template.

After changing clock/launch support, rebuild all affected packages:

```bash
colcon build --packages-select farol2_sim farol2_bringup farol2_nav farol2_allocation
source install/setup.bash
```

Clock readiness and domain launch-policy tests mock DDS transport, so they can run
without sockets. Live world/domain isolation still requires a ROS-enabled host.


### Shared environment configuration

The world YAML contains separate parameter sections for `/world/sim_clock` and
`/**/vehicle_sim`. Each vehicle simulator loads its default/selected vehicle YAML,
then the shared world YAML. Thus world settings take precedence over old vehicle
values. Both `start_vehicle_sim.launch.py` and standalone `farol2_sim.launch.py`
accept `world_config`, defaulting to `farol2_bringup/config/simple_world.yaml`.

Shared settings are `environment.fluid_density`, `gravity`, `origin.latitude`,
`origin.longitude`, `origin.altitude`, `water_surface_z`, and `current.speed` and
`current.angle`. Current is constant and horizontal: speed is nonnegative in m/s,
and angle is in degrees clockwise from north, pointing toward the flow direction
(0° north, 90° east). Its NED velocity is
`[speed * cos(angle), speed * sin(angle), 0]`, with the angle converted to radians.
It is added once during kinematics; dynamics still use through-water velocity.
The origin is geographic; vehicle
`initial_state.position` is local NED in metres. GNSS altitude uses
`origin.altitude - position.z`; sensor depth and the legacy surface-buoyancy model
use `position.z - water_surface_z`. All migrated vehicles retain their old initial
geographic location, now expressed as local `[0,0,0]`.

When using a custom world, pass the **same file** to each launch:

```bash
ros2 launch farol2_bringup start_simple_world.launch.py world_config:=/path/to/world.yaml
ros2 launch farol2_bringup start_vehicle_sim.launch.py \
  vehicle_name:=magicelectric world_config:=/path/to/world.yaml
```

For now, each simulator reads this file and computes the same constant current. The world
launcher does not broadcast environment parameters, resolve the file selected by
another launch, or publish environment updates.

## Sensors

`VehicleSimNode` constructs enabled sensors in `initialize_sensors()` and calls
their `publish()` methods after each integration step. Each sensor owns its
publishers, configuration and noise sampler. All receive the same read-only
`SensorState` (vehicle state, acceleration, environment and simulation timestamp).
The snapshot also contains actual post-step thruster RPMs and control-surface
angles (radians internally). Ground-truth topics, joint states and the world
transform remain in the node; actuator measurement publishers belong to sensors.

`sensor_list` selects the enabled sensors: `gnss`, `imu`, `dvl`, `depth_cell`,
`altimeter`, `thruster_rpm`, and `control_surface_angle`.
Unknown or duplicate names are rejected; an empty list disables all sensors.
Each sensor's parameters live under `sensor.<name>`, including its noise settings.
Only listed sensors create publishers and load their measurement parameters.
For example:

```yaml
sensor_list: [gnss, imu]
sensor:
  gnss:
    measure_velocity_over_ground: true
    measure_utm_ned: true
    noise_enabled: true
    position_noise:
      bias: [0.0, 0.0, 0.0]
      standard_deviation: [0.1, 0.1, 0.0]
    velocity_noise:
      bias: [0.0, 0.0, 0.0]
      standard_deviation: [0.01, 0.01, 0.01]
  imu:
    noise_enabled: false
    orientation_noise:
      bias: [0.0, 0.0, 0.0]
      standard_deviation: [0.0, 0.0, 0.0]
    angular_velocity_noise:
      bias: [0.0, 0.0, 0.0]
      standard_deviation: [0.0, 0.0, 0.0]
```

DVL has `mode`, `output_frame`, `noise_enabled` and `velocity_noise` settings.
Depth cell has `noise_enabled` and `position_noise` settings; it uses the down
(third) component of position noise. All noise arrays contain three components,
in the measurement's frame and units. See the complete [example YAML](../config/vehicle.yaml).

This replaces the old sensor enable booleans and shared `sensor.noise` block;
custom vehicle files must migrate too. Output topics are unchanged. GNSS owns the
geographic origin conversion and its optional UTM and ground-velocity outputs.
DVL bottom tracking uses ground velocity, while water tracking uses through-water
velocity. Shipped DVL configurations use `output_frame: sensor`, so mounting
rotations and lever-arm velocities are present in the raw measurements.

### Mounting transforms

GNSS, IMU, DVL, depth cell and the synthetic altimeter declare
`sensor.<name>.frame`, defaulting to `gnss_link`, `imu_link`, `dvl_link`,
`depth_link`, and `altimeter_link` respectively. The vehicle prefix is applied to
unqualified frame names. Before simulation starts, the node loads each enabled
sensor's `base_frame <- sensor_frame` transform from TF. Missing frames are
retried along with actuator geometry; measurements are never generated using an
assumed identity transform. Mounts are fixed and cached once, so changing a mount
requires restarting the simulator. Current shipped sensor descriptions cover all
enabled sensors; no surveyed mounting values were changed by this implementation.

For body-to-NED rotation `R`, sensor-to-body mounting rotation `Q`, body-frame
offset `r`, and body angular velocity `omega`:

```text
sensor position NED = body position NED + R * r
sensor orientation = R * Q
sensor velocity in sensor axes = Q^T * (body velocity + omega × r)
```

GNSS latitude, longitude, altitude and UTM outputs use the antenna position.
Its optional ground-velocity output uses the antenna velocity in sensor axes,
matching the message's sensor frame and navigation's inverse mounting correction.
Depth uses the cell's NED height relative to `water_surface_z`, including the
offset's vertical component after vehicle rotation. GNSS position noise remains
in NED; IMU and sensor-frame velocity noise are applied in their output axes.

The IMU quaternion describes sensor-to-NED attitude and angular velocity is in
sensor axes. Acceleration now means accelerometer **specific force**, replacing
the previous body-velocity derivative. It includes body-axis transport,
tangential and centripetal acceleration at the IMU, and gravity subtraction:

```text
specific force = Q^T * (v_water_dot + omega × v_water
  + omega_dot × r + omega × (omega × r) - R^T * [0, 0, gravity])
```

This assumes the simulator's uniform, constant NED current. At rest with an
aligned level mount, the IMU reports `[0, 0, -gravity]` in our forward/starboard/down
axes; free-fall specific force is zero. Both sensor state and acceleration are
evaluated at the completed integration step. The navigation sample-and-hold
filter can recover body attitude and body-origin velocity from these mounted
measurements; use DVL `sensor` or compensated `body` output with that filter.

The altimeter is a test signal with no seabed model or dependence on vehicle
position. It publishes `sensor_msgs/Range` on `altimeter`, remapped by the launcher
to `/<vehicle>/measurement/altimeter`, at the simulation step rate. Its range is
`3 + sin(2*pi*t/period)` metres, where `t` is simulation time in seconds and
`sensor.altimeter.period` is positive (default 20 seconds). It stays between 2 and
4 m, pauses with simulation time, and has no noise. The message uses the prefixed
`altimeter_link` frame, ultrasound type, zero field of view and dummy range limits
of 0–10 m. Its mounting TF is validated, but its synthetic range intentionally
does not change with mounting position or orientation. Enable it in `sensor_list`;
filter measurement selections are unchanged.

The actuator feedback sensors publish `farol2_interfaces/ThrusterRPM` on
`/<vehicle>/measurement/thruster_rpm` and `farol2_interfaces/ControlSurfaceAngle`
on `/<vehicle>/measurement/control_surface_angle`. RPMs come from the motor
model after command limiting, delay and lag, ordered exactly as
`vehicle.actuators.frames`. Angles come from the actual rudder state after its
limits and rate limit, converted to degrees for the message. Both use the same
simulation timestamp as other sensors and the vehicle base frame. These are
exact feedback sensors without additional parameters or measurement noise.
The angle sensor requires an enabled rudder; it currently reports one angle.
Removing its name from `sensor_list` disables angle measurements while joint
state updates for TF continue. RPM feedback is enabled for all shipped vehicles;
angle feedback is enabled for Magic Electric and the example vehicle.

Noise generators are now independent per sensor with distinct deterministic seeds
(GNSS 0, IMU 1, DVL 2, depth 3).
Enabling another sensor no longer changes a sensor's noise sequence; exact random
samples differ from the previous shared generator. Each sensor now has separate
noise parameters: `bias` and `standard_deviation` (formerly named `variance`).
The depth variance message field retains its existing value for now. Bias and
random noise are both disabled by that sensor's `noise_enabled: false`.

Independent sampling, richer covariance/validity reporting, noise semantics,
moving sensor mounts and a physical seabed model remain future work. GNSS
velocity and bottom-track DVL still share `velocity_over_ground`; disable the
GNSS velocity output when using DVL bottom tracking if both sources are unwanted.

See [the refactor checklist](refactor_plan.md) for the remaining clock reset,
integration catch-up, environment and sensor tasks.
