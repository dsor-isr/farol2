# Packages

The `farol2` repository has thirteen packages. They are grouped here by where they sit in the control chain ([architecture](../architecture.md)).

| Group | Package | Nodes (executable) | Page |
|---|---|---|---|
| Navigation | `farol2_nav` | `navigation_filter` (`filter_node`) | [nav](nav/README.md) |
| Guidance | `farol2_waypoint` | `waypoint` (`waypoint_node`) | [waypoint](waypoint/README.md) |
| Guidance | `farol2_planning` | `paths` (`path_node`) | [planning](planning/README.md) |
| Guidance | `farol2_path_following` | `path_following` (`path_following_node`) | [path_following](path_following/README.md) |
| Control | `farol2_inner_loop` | `inner_loop` (`inner_loop_node`), `open_loop` | [inner_loop](inner_loop/README.md) |
| Control | `farol2_allocation` | `actuator_allocation` (`allocation_node`) | [allocation](allocation/README.md) |
| Simulation | `farol2_sim` | `auv_sim`, `magic_electric_sim`, `sim_clock` | [sim](sim/README.md) |
| Description | `farol2_description` | `robot_state_publisher` (launch only) | [description](description/README.md) |
| Support | `farol2_interfaces` | none (messages) | [interfaces](interfaces/README.md) |
| Support | `farol2_utils` | none (C++ library) | [utils](utils/README.md) |
| Operator | `farol2_console` | `console_server`, `console_parser` (`console_parser_node`), `nav2console_state` (`nav2console_state_node`) | [console](console/README.md) |
| Bringup | `farol2_bringup` | none (launch and config) | [bringup](bringup/README.md) |
| Tooling | `farol2_cli` | none (Bash, not a ROS package) | [cli](cli/README.md) |

Each page follows the same order: what the package does, how it works, topics, services, parameters, and the known gaps that touch it.
