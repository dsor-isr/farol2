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

  launch_waypoint_arg = DeclareLaunchArgument(
    'waypoint',
    default_value='true',
    description='Boolean to determine if "waypoint" node is launched.'
  )
  
  launch_path_following_arg = DeclareLaunchArgument(
    'path_following',
    default_value='true',
    description='Boolean to determine if "path_following" node is launched.'
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
              'outer_loop.yaml'
            ]),
            
            # override with personal control configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'outer_loop.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################

  waypoint_node = Node(
    package='waypoint',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'control', 'outer_loop']),
    executable='waypoint_node',
    name='waypoint',
    output='screen',
    condition=IfCondition(LaunchConfiguration('waypoint')),
    parameters=params
  )

  path_following_node = Node(
    package='path_following',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'control', 'outer_loop']),
    executable='path_following_node',
    name='path_following',
    output='screen',
    condition=IfCondition(LaunchConfiguration('path_following')),
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
    launch_waypoint_arg,
    launch_path_following_arg,
    # nodes
    waypoint_node,
    path_following_node,
  ])