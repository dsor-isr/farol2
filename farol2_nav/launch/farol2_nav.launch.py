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
  
  filter_handler_node = Node(
    package='farol2_nav',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'nav']),
    executable='filter_handler',
    name='filter_handler',
    output='screen',
    parameters=params,
    remappings=[
      # Publishers
      ('state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ('nav_sat_fix', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/nav_sat_fix')]),
      # Services
      ('change_filter', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/change_filter')]),
    ]
  )

  sample_and_hold_filter_node = Node(
    package='farol2_nav',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'nav']),
    executable='sample_and_hold',
    name='sample_and_hold',
    output='screen',
    condition=IfCondition(LaunchConfiguration('sample_and_hold')),
    parameters=params,
    remappings=[
      # Subscribers
      ('measurement', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement')]),
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
      ('rudder_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rudder_command')]),
      # Publishers
      ('state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ('position_raw', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/position_raw')]),
      ('model_velocity', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/model_velocity')]),
      ('course_meas_debug1', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/course_meas_debug1')]),
      ('course_meas_debug2', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/course_meas_debug2')]),
      # Services
      ('tune_position_ekf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/tune_position_ekf')]),
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
    launch_sample_and_hold_arg,
    # ...
    # nodes
    # filter_handler_node,
    sample_and_hold_filter_node,
    # ... 
  ])