from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  
  ####################
  # Launch arguments #
  ####################
  vehicle_ns_arg = DeclareLaunchArgument(
    'vehicle_ns',
    default_value='vehicle0',
    description='Vehicle namespace for topics, nodes, etc.'
  )

  params_file_arg = DeclareLaunchArgument(
    'params_file',
    default_value=PathJoinSubstitution([
      FindPackageShare('pid'),
      'config',
      'pid.yaml'
    ]),
    description='Main PID parameters file.'
  )

  launch_pid_arg = DeclareLaunchArgument(
    'pid',
    default_value='true',
    description='Boolean to determine if "pid" node is launched.'
  )

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
    {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
    LaunchConfiguration('params_file'),
  ]


  ###################
  # Nodes to launch #
  ###################
  pid_node = Node(
    package='pid',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'inner_loop']),
    executable='pid_control',
    name='pid',
    output='screen',
    condition=IfCondition(LaunchConfiguration('pid')),
    parameters=params
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    params_file_arg,
    launch_pid_arg,
    # nodes
    pid_node,
  ])