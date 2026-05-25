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
              'waypoint.yaml'
            ]),

            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'waypoint.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  
  waypoint_node = Node(
    package='farol2_waypoint',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'control']),
    executable='waypoint_node',
    name='waypoint',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('mission_status', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/mission_status')]),
      ('state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('turn_radius_flag', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/turn_radius_flag')]),
      ('yaw_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/yaw')]),
      ('yaw_rate_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/yaw_rate')]),
      ('u_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/surge')]),
      ('v_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/sway')]),
      # Services
      ('wp_standard', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/waypoint/send_wp_standard')]),
      ('wp_loose', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/waypoint/send_wp_loose')]),
      ('wp_heading', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/waypoint/send_wp_heading')]),
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
    waypoint_node,
  ])