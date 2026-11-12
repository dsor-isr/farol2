# Simulation refactor plan

Implementation notes and verification commands: [simulation documentation](README.md).

The shared dynamics/actuator implementation is in place. Core tests and offline
checks of all six configurations pass; live ROS verification remains pending
because DDS sockets were blocked in the editing environment. The stored velocity
is through water; kinematics adds NED current once to obtain ground velocity. DVL
bottom/water tracking selects ground/water velocity independently of output frame.
Quadratic damping
currently supports nonnegative diagonal coefficients; inertia, added mass and
linear damping support full matrices.

This document summarizes the source review of `farol2_sim` and its integration
with bringup, allocation, vehicle descriptions, and navigation. The findings
come from inspecting the source; runtime verification remains part of the work.

## Target architecture

All vehicles should use one simulator executable and one shared Fossen dynamics
and rigid-body kinematics implementation. Vehicle configuration should select
physical parameters, constrained DOFs, actuators, and sensors.

```text
Actuator commands
    -> actuator dynamics and force models
    -> summed body wrench
    -> shared vehicle dynamics and kinematics
    -> timestamped ground-truth state
    -> individual sensor models
    -> measurement topics
```

Allocation converts desired wrench into actuator commands. The simulator's
actuator models convert commands into physical wrench. A rudder requires a
flow-dependent force model, even though it uses the same vehicle dynamics core.

## 1. Fix simulation time and clock ownership

### Status

- `start_simple_world.launch.py` owns the single clock for a selected DDS domain (default 0).
- Clock and environment settings live in `farol2_bringup/config/simple_world.yaml`,
  outside vehicle configuration. Each vehicle loads that world file for now.
- Vehicle launch sets the domain before discovery and starts the stack with
  simulation time enabled, after receiving advancing clock messages.
- Standalone simulator launch has the same clock readiness check.
- Missing/frozen or duplicate clocks produce startup errors; world startup also
  checks that its selected domain has no clock owner.
- Navigation and allocation processing use ROS-time timers. TF discovery and
  clock generation use wall timers.
- Clock readiness/domain behavior has transport-free tests; live multi-world
  validation remains pending because DDS sockets were unavailable.

### Tasks

- [x] Pass `use_sim_time=true` to the entire simulated stack.
- [x] Introduce a world-level launch that owns exactly one simulation clock.
- [x] Make per-vehicle launches use the shared clock, including external clocks.
- [x] Use ROS-time timers for navigation and allocation processing; retain wall
      timers for clock generation and startup discovery where appropriate.
- [x] Require a ready world clock before starting simulated vehicles.
- [x] Select a DDS domain per world and propagate it to joining vehicles.
- [ ] Define physics stepping, clock publication, catch-up, and overload behavior.
- [ ] Define pause, resume, reset, and backward-time-jump behavior.
- [ ] Verify two vehicles, mixed physics rates, pause, and speedup.

## 2. Unify vehicle dynamics and kinematics

### Current findings

- `auv_sim.cpp` combines a Fossen-style model, actuators, sensors, and ROS code.
- `magic_electric_sim.cpp` implements separate dynamics and planar kinematics.
- Magic Electric loads sway damping parameters but never computes sway
  acceleration.
- Magic Electric adds current during position integration, then writes ground
  velocity back into the body velocity used by its dynamics.
- The AUV model uses diagonal mass-related matrices and a spherical buoyancy
  approximation, which limit its use across different hulls.

### Tasks

- [x] Extract a ROS-independent dynamics library and a single `vehicle_sim` node.
- [x] Separate dynamics, rigid-body kinematics, and numerical integration.
- [x] Support configurable mass/inertia, added mass, damping, and restoring forces.
- [x] Support full matrices, with diagonal configuration as a convenience.
- [x] Define ground-relative and water-relative velocities consistently and
      implement current handling against that convention.
- [x] Use rotation-matrix attitude internally (updated requirement).
- [x] Support reduced DOFs through a consistent reduced solve or constraints.
      Distinguish constrained motion from underactuation.
- [ ] Replace universal spherical buoyancy with configurable CG, CB, displaced
      volume, and surface-restoring behavior.
- [x] Migrate every vehicle to the same model with explicit parameters.
- [ ] Compare Magic Electric against reference manoeuvres; do not preserve
      existing sway/current errors as desired behavior.

Reference: [Fossen's marine craft model](https://www.fossen.biz/html/marineCraftModel.html).

## 3. Separate actuator physics from vehicle physics

### Current findings

- AUV thruster lag, delay, RPM-to-force conversion, and wrench generation live
  inside the vehicle model.
- Magic Electric embeds propeller and rudder physics inside `updateState()`.
- Magic Electric defaults to fixed 660 RPM, overriding subscribed commands.
- Zero AUV thruster delay can produce an empty delay buffer that is accessed
  unconditionally.
- RPM array validation is inconsistent, and commands have no timeout.

### Tasks

- [x] Extract thruster/propeller and rudder physics into actuator source files.
- [ ] Give actuators explicit IDs, command mappings, limits, lag/delay, and state.
- [x] Have each actuator produce a body wrench for the shared dynamics core.
- [x] Compute rudder forces using local relative flow, angle, lift, and drag.
- [x] Make fixed RPM an explicit scenario option.
- [x] Fix zero-delay handling and verify delay timing.
- [ ] Validate command sizes and values, and define stale-command behavior.
- [x] Publish actuator feedback and joint states consistently.

## 4. Make geometry and TF consistent

### Current findings

- Static thrusters are configured as an ordered list of frames in YAML.
- The simulator waits for body-to-thruster TF, extracts position and the rotated
  local +X thrust axis, and caches the geometry once.
- Allocation uses a `6 x N` wrench matrix; simulation uses a separate `N x 6`
  direction/position representation.
- Shared geometry helpers currently live in `farol2_allocation`.
- Magic Electric's simulator does not use actuator TF. Its configured rudder
  lever arm is 4.0 m, while its Xacro places the rudder at -4.5 m.
- Most vehicle descriptions lack sensor frames; `myellow` includes them.

### Tasks

- [ ] Keep mounting poses and joint geometry in vehicle descriptions; keep model
      types and physical coefficients in configuration.
- [ ] Move common geometry helpers into a shared library.
- [ ] Use one geometry representation and wrench mapping across packages.
- [ ] Validate configured frames, actuator ordering, thrust axes, and units.
- [ ] Cache fixed geometry after initialization; derive moving rudder geometry
      from joint configuration and actuator state.
- [ ] Resolve Magic Electric's lever-arm discrepancy and joint/frame naming.
- [ ] Add all configured sensor frames to every vehicle description.
- [ ] Report missing required transforms through readiness/errors instead of
      silently treating them as identity.

## 5. Split sensors into independent implementations

### Current findings

- Measurements are generated inside each vehicle node's `publishMeasurements()`.
- Sensors ignore mounting transforms and publish at the physics rate.
- GNSS/depth use body-origin position; DVL ignores mounting rotation and
  rotational lever-arm velocity.
- IMU ignores mounting rotation and offset acceleration, and publishes body
  velocity derivatives as acceleration.
- Both IMU publishers omit `frame_id`; Magic Electric GNSS also omits it.
- Velocity components and their declared frames can disagree.
- Parameters named `variance` are used as Gaussian standard deviations;
  covariance reporting is mostly absent.
- Magic Electric always publishes water-relative velocity without a sensor
  enable flag.

### Tasks

- [x] Create individual IMU, GNSS, DVL, and depth `.cpp` implementations with
      declarations in a shared `sensors.hpp`.
- [x] Give each sensor its own configuration and publishers, selected by the
      existing enable flags.
- [ ] Give each sensor an independent sampling rate.
- [x] Feed sensors an immutable, timestamped ground-truth snapshot.
- [x] Apply mounting position, orientation, and rotational lever-arm effects for fixed mounts.
- [x] Define the IMU output convention and implement the corresponding
      accelerometer model, including gravity and offset effects.
- [ ] Define precise frame and unit contracts for every measurement topic.
- [ ] Populate frame IDs, timestamps, covariance, and validity fields consistently.
- [ ] Correct variance/standard-deviation semantics and use independent,
      deterministic random seeds.
- [ ] Support configurable latency and appropriate sensor validity/dropout rules.

Separate source files do not require separate processes. Initially, sensor
objects can share a process and simulation scheduler while owning their own
measurement generation and publishing.

## 6. Align simulator and navigation contracts

### Current findings

- Navigation already applies mounting corrections in `sample_and_hold.cpp`,
  whereas simulated measurements omit those mounting effects.
- GNSS velocity is published in NED components, but navigation interprets the
  vector in sensor axes.
- DVL can publish body or inertial components while retaining `dvl_link` as its
  frame ID.
- Navigation's TF helper falls back to identity on missing transforms.

### Tasks

- [ ] Agree on sensor-frame versus inertial-frame output per topic and update
      publishers and consumers together.
- [ ] Ensure mounting corrections are applied exactly once in each direction.
- [ ] Test rotated sensors and nonzero mounting offsets end to end.
- [ ] Keep truth separate from estimated navigation state; publish a coherent
      truth message with pose, velocities, accelerations, current, and timestamp.
- [ ] Define shared world/NED/body conventions and geographic origin, including
      any visualization-frame conversions.

## 7. Simplify bringup, configuration, and package maintenance

### Current findings

- Bringup selects simulator executables using `vehicle_name == 'magicelectric'`.
- Simulator override loading is commented out, but the clock honors
  `config_to_use`; allocation is not passed that selection by stack bringup.
- AUV YAML uses densities of `1.025` and `1.0`, while calculations combine them
  with kilogram masses and geometric lengths.
- Automatic vehicle IDs depend on a short ROS discovery wait and can race.
- Bringup documentation describes paths and clock behavior that disagree with
  the implementation.
- Package manifests omit dependencies used by launch/build code.

### Tasks

- [x] Remove vehicle-name branches and launch the same simulator for all vehicles.
- [x] Move clock, environment and geographic origin settings into shared world YAML.
- [ ] Generate and publish shared current conditions from the world.
- [ ] Load defaults and selected overrides consistently across the stack.
- [ ] Forward configuration selections to allocation.
- [ ] Standardize SI units, especially density in kg/m³, and recalibrate buoyancy.
- [ ] Validate parameter sizes, finite values, rates, matrices, limits, and frames
      before declaring a vehicle ready.
- [ ] Use explicit unique IDs for configured fleets.
- [ ] Add readiness checks rather than relying on launch-list order.
- [ ] Correct documentation and dependency declarations.
- [ ] Add focused automated checks for physics, actuator geometry, sensor
      transforms, configuration loading, and simulation launch behavior.

## Suggested implementation order

1. **Clock and configuration fixes:** establish one timeline and reliable
   configuration selection before comparing model behavior.
2. **Shared physics and actuator extraction:** create the common core, resolve
   velocity conventions, and migrate vehicle dynamics.
3. **Geometry, sensors, and navigation contracts:** implement mounting effects
   and independent sensor sampling together with consumer changes.
4. **Vehicle migration and validation:** check all vehicle configurations,
   reference manoeuvres, multi-vehicle operation, and documentation.

## Acceptance checks

- [x] Every vehicle launches the same dynamics implementation.
- [ ] Exactly one `/clock` publisher drives a multi-vehicle simulation.
- [ ] Simulated processing follows pause and speedup consistently.
- [ ] Thruster/rudder forces and moments agree with configured geometry.
- [ ] Zero-delay actuators and invalid commands behave predictably.
- [x] Current does not accumulate spuriously in the vehicle state.
- [ ] Rotated and offset sensors reconstruct the expected body state through
      navigation when noise is disabled.
- [ ] Sensor rates, timestamps, frames, covariance, and noise match configuration.
- [ ] Selected configuration overrides reach every intended subsystem.
