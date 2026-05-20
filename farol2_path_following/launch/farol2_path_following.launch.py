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
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'path_following.yaml'
            ]),

            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'path_following.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  path_following_node = Node(
    package='farol2_path_following',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'control', 'path_following']),
    executable='path_following_node',
    name='path_following',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('mission_status', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/mission_status')]),
      ('state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ('path_data', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/path_data')]),
      ('vc', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/vc')]),
      # Publishers
      ('surge', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/surge')]),
      ('sway', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/sway')]),
      ('yaw', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/yaw')]),
      ('yaw_rate', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/yaw_rate')]),
      ('rabbit', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/gamma')]),
      ('observer_x', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/current/x')]),
      ('observer_y', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/current/y')]),
      ('pfollowing_debug', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/debug')]),
      # Services
      # Service names to start and stop the path following algorithm
      ('start_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/Start')]),
      ('stop_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/Stop')]),
      ('update_gains_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/UpdateGains')]),
      ('reset_vt_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/ResetVT')]),
          # Service names to switch the path following algorithm
      ('marcelo_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetMarcelo')]),
      ('aguiar_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetAguiar')]),
      ('breivik_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetBreivik')]),
      ('fossen_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetFossen')]),
      ('romulo_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetRomulo')]),
      ('lapierre_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetLapierre')]),
      ('pramod_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetPramod')]),
      ('ravi_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetRavi')]),
      ('samson_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetSamson')]),
      ('ilos_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetIlos')]),
      ('relative_heading_pf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/SetRelativeHeading')]),
      # Service names related to the path
      ('reset_path', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/ResetPath')]),
      ('set_path_mode', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/SetMode')]),
      # Service names related to the waypoint
      ('wp_standard', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/waypoint/send_wp_standard')]),
      # Service for resetting Dead Reckoning in filter
      ('reset_dr', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/reset_dead_reckoning')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    vehicle_ns_arg,
    vehicle_name_arg,
    use_sim_time_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    path_following_node,
  ])