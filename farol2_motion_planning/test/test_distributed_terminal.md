# mred0
ros2 service call /mred0/set_states farol2_motion_planning/srv/SetState "{vehicle_name: 'mred0', e: 0.0, n: 0.0, theta: 0.0, v: 0.05}"
ros2 service call /mred0/set_goal farol2_motion_planning/srv/SetState "{vehicle_name: 'mred0', e: 0.0, n: 200.0, theta: 0.0, v: 0.05}"
ros2 param set /farol2_motion_planning/distributed_planner_node/mred0 discovery_mode true

# myellows0
ros2 service call /myellow0/set_states farol2_motion_planning/srv/SetState "{vehicle_name: 'myellow0', e: 2.0, n: 200.0, theta: 3.14, v: 0.05"
ros2 service call /myellow0/set_goal farol2_motion_planning/srv/SetState "{vehicle_name: 'myellow0', e: 0.0, n: 0.0, theta: 3.14, v: 0.05}"
ros2 param set /farol2_motion_planning/distributed_planner_node/myellow0 discovery_mode true


# after they found each other
ros2 param set /farol2_motion_planning/distributed_planner_node/mred0 discovery_mode false
ros2 param set /farol2_motion_planning/distributed_planner_node/myellow0 discovery_mode false

# To start optimizing
ros2 service call /mred0/start_distributed  std_srvs/srv/Trigger "{}"
ros2 service call /myellow0/start_distributed  std_srvs/srv/Trigger "{}"


