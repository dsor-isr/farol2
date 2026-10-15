# FAROL 2 documentation

FAROL 2 is DSOR's (ISR, Instituto Superior Técnico) ROS 2 Jazzy stack for guidance, navigation and control of marine vehicles: Medusa-class AUVs/ASVs, BlueROV2, the DELFIM catamaran and the magicelectric surface vessel. It is the ROS 2 port of the ROS 1 [FAROL](https://github.com/dsor-isr/farol) stack.

These pages describe the `dev` branch of [`dsor-isr/farol2`](https://github.com/dsor-isr/farol2) at commit `693f523` (2026-07-30), checked on 2026-10-15 by building the workspace and running the simulator. They complete and reorganise the per-package `docs/` folders in the repository, which are partly out of date.

## Start here

| Page | Read it to |
|---|---|
| [Getting started](getting-started.md) | Install, build, run a simulated vehicle and send a waypoint or a path |
| [Architecture](architecture.md) | See how the packages connect: control chain, `mission_status`, every topic and service, frames and units |
| [Configuration](configuration.md) | Find which YAML configures which node, the configured vehicles, and how to add a vehicle |
| [Repositories](repositories.md) | Know which repositories and branches make up the stack |
| [MATLAB and Simulink](matlab.md) | Use the magicelectric identification scripts and the Simulink ROS 2 simulation |
| [Known gaps](GAPS.md) | Review 41 problems found in the code, with severity, evidence and file references |

## Packages

| Package | Role | Page |
|---|---|---|
| `farol2_nav` | Navigation filter pipeline → `NavigationState` | [nav](packages/nav/README.md) |
| `farol2_waypoint` | Go-to-point outer loop | [waypoint](packages/waypoint/README.md) |
| `farol2_planning` | Parametric paths (lines, arcs, circles, lemniscates) | [planning](packages/planning/README.md) |
| `farol2_path_following` | Eleven path-following laws | [path_following](packages/path_following/README.md) |
| `farol2_inner_loop` | Per-axis PID controllers, open-loop surge | [inner_loop](packages/inner_loop/README.md) |
| `farol2_allocation` | Wrench → thruster RPM and rudder angle | [allocation](packages/allocation/README.md) |
| `farol2_sim` | AUV and magicelectric simulators, `/clock` | [sim](packages/sim/README.md) |
| `farol2_description` | URDFs and static TF | [description](packages/description/README.md) |
| `farol2_interfaces` | Shared messages | [interfaces](packages/interfaces/README.md) |
| `farol2_utils` | Filters, angle and TF helpers | [utils](packages/utils/README.md) |
| `farol2_console` | Web console bridge and mission parser | [console](packages/console/README.md) |
| `farol2_bringup` | Launch files and per-vehicle configs | [bringup](packages/bringup/README.md) |
| `farol2_cli` | `farol` shell helper (build, run, drivers) | [cli](packages/cli/README.md) |

## Most important gaps

The full list is in [GAPS.md](GAPS.md). The ones that change vehicle behaviour today:

1. Every vehicle's `nav.yaml` is ignored because its key does not match the node name, so navigation runs with defaults ([#1](GAPS.md#1-every-vehicles-navyaml-is-silently-ignored-high)).
2. The simulator runs on `/clock` while the stack uses wall time ([#2](GAPS.md#2-simulation-and-stack-run-on-different-clocks-high)).
3. `open_loop` and `actuator_allocation` both publish `rpm_command` on some vehicles ([#3](GAPS.md#3-two-nodes-publish-to-the-same-rpm_command-topic-high)).
4. GNSS velocity in NED is used as body-frame velocity ([#4](GAPS.md#4-gnss-velocity-ned-is-treated-as-a-body-frame-velocity-high)), and the surge loop has no feedback on most vehicles ([#5](GAPS.md#5-surge-loop-has-no-feedback-on-most-vehicles-high)).
5. The standard waypoint always commands 0.3 m/s, so vehicles never stop ([#36](GAPS.md#36-standard-waypoint-never-stops-the-vehicle-high)).
6. Rudder allocation mixes degrees and radians ([#35](GAPS.md#35-rudder-allocation-mixes-degrees-and-radians-and-ignores-flow-speed-high)).
7. On real vehicles, `dev` cannot command magicelectric's rudder ([#38](GAPS.md#38-magicelectric-rudder-topics-no-longer-match-the-drivers-high)) or any Medusa thruster ([#39](GAPS.md#39-medusa-thruster-drivers-cannot-receive-rpm_command-high)).
8. `dsor_simulation-ros2` is stale and launches packages that no longer exist ([#6](GAPS.md#6-dsor_simulation-ros2-launches-packages-that-no-longer-exist-high)).

## Building this site

The pages are plain Markdown and read fine on GitHub. To serve them as a site, like the ROS 1 FAROL docs:

```bash
pip install mkdocs-material
mkdocs serve        # from the folder that holds mkdocs.yml (one level above these pages)
```

## Conventions used in these pages

- `/<ns>` is the vehicle namespace, `<vehicle_name><vehicle_id>` (for example `/mred0`). Topic names in tables are relative to it.
- `file:line` references are relative to the `farol2` repository root at `693f523`.
- Statements marked "inferred" were not checked by running code.
