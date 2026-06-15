from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
  ####################
  # Launch arguments #
  ####################
  vehicle_id_arg = DeclareLaunchArgument(
    'vehicle_id',
    default_value='0',
    description='Vehicle ID'
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

  config_to_use = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config folder to use.'
  )

  vehicle_ns = PythonExpression(["'", LaunchConfiguration('vehicle_name'), "' + '", LaunchConfiguration('vehicle_id'), "'"])

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_name': LaunchConfiguration('vehicle_name')},
            {'vehicle_id': LaunchConfiguration('vehicle_id')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            
            # load default console configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config',
              LaunchConfiguration('vehicle_name'),
              'default',
              'addons.yaml'
            ]),
            # override config — same path as default when config_to_use == 'default'
            # PathJoinSubstitution([
            #   FindPackageShare('farol2_bringup'),
            #   'config',
            #   LaunchConfiguration('vehicle_name'),
            #   LaunchConfiguration('config_to_use'),
            #   'addons.yaml'
            # ]),
          ]


  ###################
  # Nodes to launch #
  ###################

  console_parser_node = Node(
    package='farol2_console',
    namespace=PathJoinSubstitution([vehicle_ns, 'addons']),
    executable='console_parser_node',
    name='console_parser',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('Mission_String', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/addons/Mission_String')]),
      ('state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      ('mission_status', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/mission_status')]),
      # Publishers
      ('Path_Section', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/addons/path_section')]),
      ('Formation', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/addons/formation')]),
      ('biased_formation', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/addons/biased_formation')]),
      ('WPRef', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/addons/WPRef')]),
      ('DepthRef', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/depth')]),
      ('AltRef', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/altitude')]),
      ('FullMission', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/addons/full_mission')]),
      # Services
      ('reset_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/ResetPath')]),
      ('arc2d_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SpawnArc2DPath')]),
      ('line_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SpawnLinePath')]),
      ('set_speed', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SetConstVdVehicle')]),
      ('pf_start', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/Start')]),
      ('pf_stop', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/Stop')]),
    ]
  )

  console_server_node = Node(
    package='farol2_console',
    namespace=PathJoinSubstitution([vehicle_ns, 'addons']),
    executable='console_server',
    name='console_server',
    output='screen',
    additional_env={
      'PYTHONWARNINGS': "ignore:'cgi' is deprecated and slated for removal in Python 3.13:DeprecationWarning,ignore:'cgitb' is deprecated and slated for removal in Python 3.13:DeprecationWarning",
    },
    parameters=params,
    remappings=[
      # Services
      ('wp_standard', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/waypoint/send_wp_standard')]),
    ],
  )

  nav2console_state_node = Node(
    package='farol2_console',
    namespace=PathJoinSubstitution([vehicle_ns, 'addons']),
    executable='nav2console_state_node',
    name='nav2console_state',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('nav_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('console_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/State')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_id_arg,
    vehicle_name_arg,
    use_sim_time_arg,
    config_to_use,
    # nodes
    console_parser_node,
    console_server_node,
    nav2console_state_node
  ])