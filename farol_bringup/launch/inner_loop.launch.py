from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
  
  ####################
  # Launch arguments #
  ####################
  vehicle_ns_arg = DeclareLaunchArgument(
    'vehicle_ns',
    default_value='vehicle0',
    description='Vehicle namespace for topics, nodes, etc.'
  )

  vehicle_name_arg = DeclareLaunchArgument(
    'vehicle_name',
    default_value='vehicle',
    description='Vehicle name'
  )

  config_package_path_share_arg = DeclareLaunchArgument(
    'config_package_path_share',
    default_value='',
    description='Path to the config package, usually the personal bringup of the workspace, in the install folder'
  )

  config_package_path_real_arg = DeclareLaunchArgument(
    'config_package_path_real',
    default_value='',
    description='Path to the config package, usually the personal bringup of the workspace, in the src folder'
  )

  launch_pid_arg = DeclareLaunchArgument(
    'pid',
    default_value='true',
    description='Boolean to determine if "pid" node is launched.'
  )

  launch_open_loop_arg = DeclareLaunchArgument(
    'open_loop',
    default_value='false',
    description='Boolean to determine if "open_loop" node is launched.'
  )

  launch_rudder_arg = DeclareLaunchArgument(
    'rudder',
    default_value='true',
    description='Boolean to determine if "rudder" node is launched.'
  )

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},

            # load default ROS configurations (from tmp files)
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_real'),
              'config_personal',
              '.ros_tmp',
              PythonExpression(["'default_ros_' + '", LaunchConfiguration('vehicle_ns'), "' + '.yaml'"])
            ]),

            # override default with personal ROS configurations (from tmp files)
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_real'),
              'config_personal',
              '.ros_tmp',
              PythonExpression(["'personal_ros_' + '", LaunchConfiguration('vehicle_ns'), "' + '.yaml'"])
            ]),

            # load default control configs
            PathJoinSubstitution([
              FindPackageShare('farol_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'inner_loop.yaml'
            ]),
            
            # override with personal control configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'inner_loop.yaml'
            ]),
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

  open_loop_node = Node(
    package='open_loop',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'inner_loop']),
    executable='open_loop_control',
    name='open_loop',
    output='screen',
    condition=IfCondition(LaunchConfiguration('open_loop')),
    parameters=params
  )

  rudder_node = Node(
    package='rudder',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'inner_loop']),
    executable='rudder',
    name='rudder',
    output='screen',
    condition=IfCondition(LaunchConfiguration('rudder')),
    parameters=params
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    vehicle_name_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    launch_pid_arg,
    launch_open_loop_arg,
    launch_rudder_arg,
    # nodes
    pid_node,
    open_loop_node,
    rudder_node,
  ])