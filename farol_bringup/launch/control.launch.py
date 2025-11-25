from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
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
              'control.yaml'
            ]),
            
            # override with personal control configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'control.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  pid_node = Node(
    package='pid',
    namespace=[LaunchConfiguration('vehicle_ns'), '/control', '/inner_loop'],
    executable='pid_control',
    name='pid',
    output='screen',
    parameters=params
  )

  waypoint_node = Node(
    package='waypoint',
    namespace=[LaunchConfiguration('vehicle_ns'), '/control', '/outer_loop'],
    executable='waypoint_node',
    name='waypoint',
    output='screen',
    parameters=params
  )

  path_following_node = Node(
    package='path_following',
    namespace=[LaunchConfiguration('vehicle_ns'), '/control', '/outer_loop'],
    executable='path_following_node',
    name='path_following',
    output='screen',
    parameters=params
  )

  open_loop_node = Node(
    package='open_loop',
    namespace=[LaunchConfiguration('vehicle_ns'), '/control', '/inner_loop'],
    executable='open_loop_control',
    name='open_loop',
    output='screen',
    parameters=params
  )

  rudder_control_node = Node(
    package='rudder',
    namespace=[LaunchConfiguration('vehicle_ns'), '/control', '/inner_loop'],
    executable='rudder_control',
    name='rudder',
    output='screen',
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
    # nodes
    pid_node,
    waypoint_node,
    path_following_node,
    open_loop_node,
    rudder_control_node,
  ])