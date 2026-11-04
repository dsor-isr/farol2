# farol2_description

Robot description (URDF/xacro) for the FAROL 2 vehicles and the static TF launch.

## Dependencies
```bash
sudo apt-get install -y ros-jazzy-xacro
```

## Contents
- `urdf/<vehicle>.xacro`: `bluerov`, `delfim`, `magicelectric`, `mblack`, `mred`, `myellow`. Each defines `base_link` and the actuator frames named in `farol2_bringup/config/<vehicle>/default/allocation.yaml` (for example `thruster_portside_link`, `thruster_starboard_link` for delfim; `thruster_middle_link` and `rudder_middle_link` for magicelectric).
- `launch/tf_static.launch.py`: runs `robot_state_publisher` for the vehicle with `frame_prefix = <ns>/` (so frames are `delfim0/base_link`, ...). It takes `vehicle_name`, `vehicle_id`, `use_sim_time` and is included by `start_farol2.launch.py`.

There is no `meshes/` or `config/` directory.

## Notes
- Nodes that need a sensor-to-body transform use `farol2_utils::StaticTransformLookup`; if a frame is missing in the xacro they continue with identity and log a warning. **None of the xacro files define sensor frames** (`gnss_link`, `dvl_link`, `depth_link`, `rudder_link`, ... which the simulators use in their message headers), so on every vehicle the navigation filter logs `Static transform '<ns>/gnss_link' -> '<ns>/base_link' is not available, using identity` and treats all sensors as located at `base_link`.
- `allocation_node` and `auv_sim` wait for every thruster frame, so a wrong frame name in `allocation.yaml` keeps the vehicle silent (`Waiting for transform ...`).
- The simulators publish the TF `world -> <ns>/base_link` (the parent frame name `world` has no vehicle prefix).
