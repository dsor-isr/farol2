from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            {'vehicle_name': LaunchConfiguration('vehicle_name')},
            {'vehicle_id': LaunchConfiguration('vehicle_id')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config',
              LaunchConfiguration('vehicle_name'),
              'default',
              'path_following.yaml'
            ]),
            # override config — same path as default when config_to_use == 'default'
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config',
              LaunchConfiguration('vehicle_name'),
              LaunchConfiguration('config_to_use'),
              'path_following.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  path_following_node = Node(
    package='farol2_path_following',
    namespace=PathJoinSubstitution([vehicle_ns, 'control']),
    executable='path_following_node',
    name='path_following',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('mission_status', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/mission_status')]),
      ('state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      ('path_data', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/path_data')]),
      ('vc', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/vc')]),
      # Publishers
      ('surge', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/surge')]),
      ('sway', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/sway')]),
      ('yaw', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/yaw')]),
      ('yaw_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/yaw_rate')]),
      ('rabbit', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/gamma')]),
      ('observer_x', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/current/x')]),
      ('observer_y', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/current/y')]),
      ('pfollowing_debug', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/debug')]),
      # Services
      # Service names to start and stop the path following algorithm
      ('start_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/Start')]),
      ('stop_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/Stop')]),
      ('update_gains_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/UpdateGains')]),
      ('reset_vt_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/ResetVT')]),
          # Service names to switch the path following algorithm
      ('marcelo_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetMarcelo')]),
      ('aguiar_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetAguiar')]),
      ('breivik_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetBreivik')]),
      ('fossen_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetFossen')]),
      ('romulo_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetRomulo')]),
      ('lapierre_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetLapierre')]),
      ('pramod_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetPramod')]),
      ('ravi_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetRavi')]),
      ('samson_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetSamson')]),
      ('ilos_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetIlos')]),
      ('relative_heading_pf', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/SetRelativeHeading')]),
      # Service names related to the path
      ('reset_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/ResetPath')]),
      ('set_path_mode', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SetMode')]),
      # Service names related to the waypoint
      ('wp_standard', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/waypoint/send_wp_standard')]),
      # Service for resetting Dead Reckoning in filter
      ('reset_dr', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/reset_dead_reckoning')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    vehicle_id_arg,
    vehicle_name_arg,
    use_sim_time_arg,
    config_to_use,
    path_following_node,
  ])