from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
  config_to_use = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config folder to use.'
  )

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )

  params = [
    {'use_sim_time': LaunchConfiguration('use_sim_time')},
    {'config_folder': LaunchConfiguration('config_to_use')},
    PathJoinSubstitution([
      FindPackageShare('farol2_motion_planning'),
      'config',
      'config.yaml'
    ]),
  ]


  ###################
  # Nodes to launch #
  ###################

  interactive_planner_node = Node(
    package='farol2_motion_planning',
    namespace='farol2_motion_planning',
    executable='farol2_motion_planning_node',
    name='interactive_planner_node',
    output='screen',
    parameters=params,
    remappings=[
      # Service remappings (if needed to override defaults)
      # Publishers
      ('node_alive', '/farol2_motion_planning/node_alive'),
      ('mission_log', '/farol2_motion_planning/mission_log'),
    ]
  )


  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    use_sim_time_arg,
    config_to_use,
    # nodes
    interactive_planner_node,
  ])