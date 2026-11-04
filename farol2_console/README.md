# farol2_console

Bridge between the DSOR web console and the FAROL 2 stack. The console itself (web front end) is not in this repository.

## Nodes (all launched by `launch/farol2_console.launch.py`, namespace `/<ns>`, included by `start_farol2.launch.py`)
| Executable | ROS node name | Role |
|---|---|---|
| `console_parser_node` | `console_parser` | Parses console missions (`Mission_String`, path sections, formations, `WPRef`, `FullMission`, `DepthRef`, `AltRef`) into service calls: waypoint, `ResetPath`, `SpawnLinePath`, `SpawnArc2DPath`, `SetConstVdVehicle`, `path_following/Start`/`Stop`. |
| `console_server` (`scripts/console_server.py`) | `console_server` | Python HTTP server that exposes the vehicle to the web console (port `PORT: 7080` in `addons.yaml`). |
| `nav2console_state_node` | `nav2console_state` | Converts `nav/filter/state` into `farol2_interfaces/StateConsole` on `/<ns>/State` (console uses ENU: x east, y north). |

Topics (under `/<ns>/addons/`): `Mission_String`, `path_section`, `formation`, `biased_formation`, `WPRef`, `full_mission`; plus the inner-loop references `inner_loop/ref/depth` and `ref/altitude`.

Parameters: `farol2_bringup/config/<vehicle>/default/addons.yaml` (keys `console_parser`, `console_server`).

## Status and known problems
- Partly ported from ROS 1. `console_server.py` still carries ROS 1 code: commented `rospy`/`rosgraph` calls, `rostopic`-style help text, `cgi.FieldStorage` and `urllib` (`cgi` is removed in Python 3.13; the launch file suppresses its deprecation warnings), and the function `parseXMLStateConsoleAttributesFromROS2NamingToROS1` kept for compatibility with the web add-ons.
- `scripts/__innit__.py` is a misspelled `__init__.py`.
- The launch file loads `addons.yaml` only from `default/` (the `config_to_use` override is commented out).
- Depends on `python3-yaml`.
