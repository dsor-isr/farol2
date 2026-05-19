from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
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

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            
            # load default console configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'addons.yaml'
            ]),
            
            # override with personal console configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'addons.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################

  console_parser_node = Node(
    package='farol2_console',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'addons']),
    executable='console_parser_node',
    name='console_parser',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('Mission_String', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/addons/Mission_String')]),
      ('state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ('mission_status', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/mission_status')]),
      # Publishers
      ('Path_Section', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/addons/path_section')]),
      ('Formation', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/addons/formation')]),
      ('biased_formation', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/addons/biased_formation')]),
      ('WPRef', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/addons/WPRef')]),
      ('DepthRef', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/depth')]),
      ('AltRef', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/altitude')]),
      ('FullMission', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/addons/full_mission')]),
      # Services
      ('reset_path', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/ResetPath')]),
      ('arc2d_path', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/SpawnArc2DPath')]),
      ('line_path', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/SpawnLinePath')]),
      ('set_speed', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/SetConstVdVehicle')]),
      ('pf_start', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/Start')]),
      ('pf_stop', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/Stop')]),
    ]
  )

  console_server_node = Node(
    package='farol2_console',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'addons']),
    executable='console_server',
    name='console_server',
    output='screen',
    parameters=params,
    remappings=[
      # Services
      ('wp_standard', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/waypoint/send_wp_standard')]),
    ],
  )

  nav2console_state_node = Node(
    package='farol2_console',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'addons']),
    executable='nav2console_state_node',
    name='nav2console_state',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('nav_state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('console_state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/State')]),
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
    # nodes
    console_parser_node,
    console_server_node,
    nav2console_state_node
  ])