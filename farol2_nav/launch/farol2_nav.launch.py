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

  launch_filter_node_arg = DeclareLaunchArgument(
    'filter_node',
    default_value='false',
    description='Boolean to determine if the new side-by-side filter_node is launched.'
  )



  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

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

  filter_node = Node(
    package='farol2_nav',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'nav']),
    executable='filter_node',
    name='filter_node',
    output='screen',
    condition=IfCondition(LaunchConfiguration('filter_node')),
    parameters=params,
    remappings=[
      # Subscribers
      ('imu', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/imu')]),
      ('gnss', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/gnss')]),
      ('ned_utm', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/ned_utm')]),
      ('velocity_over_ground', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/velocity_over_ground')]),
      ('velocity_through_water', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/velocity_through_water')]),
      ('current_velocity', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/current_velocity')]),
      ('depth', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/depth')]),
      ('altimeter', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/altimeter')]),
      ('altitude', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement/altitude')]),
      ('rudder_angle', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/drivers/can_instrumentation/rudder_angle')]),
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
      # Publishers
      ('state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ##############################################################################################################################
      ## this needs to be like this because the rcl library doesnt support wildcards in remaping.
      ##  it is only possible via the cli. if it changes in the future it should work like this:
      ##
      # ('/**/state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/\\1/state')]),
      ##
      ## instead we roll like this, every new filter plugin needs to be added here manually:
      ##############################################################################################################################
      ('sample_and_hold/state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/sample_and_hold/state')]),
      ('position_current_ekf/state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/position_current_ekf/state')]),
      ('yaw_rate_ekf/state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/yaw_rate_ekf/state')]),
    ]
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
    launch_filter_node_arg,
    # ...
    # nodes
    # filter_handler_node,
    # sample_and_hold_filter_node,
    filter_node,
    # ... 
  ])