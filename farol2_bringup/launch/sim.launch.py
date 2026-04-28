from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

  launch_magic_electric_sim_arg = DeclareLaunchArgument(
    'magic_electric_sim',
    default_value='false',
    description='Boolean to determine if magic_electric_sim node is launched.'
  )

  launch_auv_sim_arg = DeclareLaunchArgument(
    'auv_sim',
    default_value='false',
    description='Boolean to determine if auv_sim node is launched.'
  )

  ####################################################
  # Define shared parameters for all simulation nodes #
  ####################################################
  common_params = [
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

            # load default sim configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'sim.yaml'
            ]),
            
            # override with personal sim configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'sim.yaml'
            ]),
          ]

  vehicle_sim_params = common_params + [
    {'use_sim_time': False}
  ]

  ###################
  # Nodes to launch #
  ###################
  auv_sim_node = Node(
    package='farol2_vehicle_sim',
    namespace=[LaunchConfiguration('vehicle_ns'), '/sim'],
    executable='auv_sim',
    name='auv_sim',
    output='screen',
    condition=IfCondition(LaunchConfiguration('auv_sim')),
    parameters=vehicle_sim_params
  )

  magic_electric_sim_node = Node(
    package='farol2_vehicle_sim',
    namespace=[LaunchConfiguration('vehicle_ns'), '/sim'],
    executable='magic_electric_sim',
    name= 'magic_electric_sim',
    output='screen',
    condition=IfCondition(LaunchConfiguration('magic_electric_sim')),
    parameters=vehicle_sim_params
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
    launch_magic_electric_sim_arg,
    launch_auv_sim_arg,
    # nodes
    auv_sim_node,
    magic_electric_sim_node,
  ])