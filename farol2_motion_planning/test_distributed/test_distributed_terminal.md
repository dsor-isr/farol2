## Manual test for mred0 and myellow0
# mred0
ros2 service call /mred0/set_states farol2_motion_planning/srv/SetState "{vehicle_name: 'mred0', e: 0.0, n: 0.0, theta: 0.0, v: 0.05}"
ros2 service call /mred0/set_goal farol2_motion_planning/srv/SetState "{vehicle_name: 'mred0', e: 0.0, n: 200.0, theta: 0.0, v: 0.05}"
ros2 param set /farol2_motion_planning/distributed_planner_node/mred0 discovery_mode true

# myellow0
ros2 service call /myellow0/set_states farol2_motion_planning/srv/SetState "{vehicle_name: 'myellow0', e: 2.0, n: 200.0, theta: 3.14, v: 0.05}"
ros2 service call /myellow0/set_goal farol2_motion_planning/srv/SetState "{vehicle_name: 'myellow0', e: 0.0, n: 0.0, theta: 3.14, v: 0.05}"
ros2 param set /farol2_motion_planning/distributed_planner_node/myellow0 discovery_mode true


# after they found each other
ros2 param set /farol2_motion_planning/distributed_planner_node/mred0 discovery_mode false
ros2 param set /farol2_motion_planning/distributed_planner_node/myellow0 discovery_mode false

# To start optimizing
ros2 service call /mred0/start_distributed  std_srvs/srv/Trigger "{}"
ros2 service call /myellow0/start_distributed  std_srvs/srv/Trigger "{}"



## Automated test with the Python script

Launch one node per vehicle, each in its own terminal:

```bash
ros2 launch farol2_motion_planning distributed_planner.launch.py vehicle_name:=mred0
ros2 launch farol2_motion_planning distributed_planner.launch.py vehicle_name:=myellow0
# one per vehicle
```

Then run the script with the same vehicle names:

```bash
python3 /##path_to_test_folder##/test_distributed_20.py mred0 myellow0 --mode opposite --seed 67
```

Arguments:

- `names`: vehicle names, which must match the launched nodes.
- `--mode opposite|circle`: default `opposite`.
  - `opposite`: starts equally spaced on a circle of radius 20 m, heading to the center. Each goal is the opposite point on the same circle.
  - `circle`: same starts. Goals are equally spaced on a circle of radius 20 m centered at (60, 0).
- `--seed`: seed for the random jitter on start and goal poses. Optional.

What the script does:

1. Waits for `/<name>/set_states` and `/<name>/set_goal` (30 s timeout per service).
2. Calls `set_states` and `set_goal` for each vehicle and sets `discovery_mode` to `true`.
3. Waits 5 s, then sets `discovery_mode` to `false`.
4. Publishes once to `/farol2_motion_planning/distributed_planner_node/start_stop_distributed` (`DistributedCommand`, `start: true`) with all vehicle names.