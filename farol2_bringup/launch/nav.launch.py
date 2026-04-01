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

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
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


  launch_sample_and_hold_arg = DeclareLaunchArgument(
    'sample_and_hold',
    default_value='true',
    description='Boolean to determine if "sample_and_hold" node is launched.'
  )

  launch_low_pass_arg = DeclareLaunchArgument(
    'low_pass',
    default_value='false',
    description='Boolean to determine if "low_pass" node is launched.'
  )

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},

            {'use_sim_time': True},

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

            # load default nav configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'nav.yaml'
            ]),
            
            # override with personal nav configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'nav.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  filter_handler_node = Node(
    package='nav_filters',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'nav']),
    executable='filter_handler',
    name='filter_handler',
    output='screen',
    parameters=params
  )

  sample_and_hold_filter_node = Node(
    package='nav_filters',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'nav']),
    executable='sample_and_hold',
    name='sample_and_hold',
    output='screen',
    condition=IfCondition(LaunchConfiguration('sample_and_hold')),
    parameters=params
  )

  low_pass_filter_node = Node(
    package='nav_filters',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'nav']),
    executable='low_pass',
    name='low_pass',
    output='screen',
    condition=IfCondition(LaunchConfiguration('low_pass')),
    parameters=params
  )


  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    vehicle_name_arg,
    use_sim_time_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    launch_sample_and_hold_arg,
    launch_low_pass_arg,
    # ...
    # nodes
    filter_handler_node,
    sample_and_hold_filter_node,
    low_pass_filter_node,
    # ... 
  ])