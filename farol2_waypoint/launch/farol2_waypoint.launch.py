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
              'waypoint.yaml'
            ]),
            # override config — same path as default when config_to_use == 'default'
          #   PathJoinSubstitution([
          #     FindPackageShare('farol2_bringup'),
          #     'config',
          #     LaunchConfiguration('vehicle_name'),
          #     LaunchConfiguration('config_to_use'),
          #     'waypoint.yaml'
          #   ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  
  waypoint_node = Node(
    package='farol2_waypoint',
    namespace=PathJoinSubstitution([vehicle_ns, 'control', 'waypoint']),
    executable='waypoint_node',
    name='waypoint',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('mission_status', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/mission_status')]),
      ('state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('turn_radius_flag', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/turn_radius_flag')]),
      ('yaw_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/yaw')]),
      ('yaw_rate_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/yaw_rate')]),
      ('u_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/surge')]),
      ('v_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/sway')]),
      # Services
      ('wp_standard', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/control/waypoint/send_wp_standard')]),
      ('wp_loose', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/control/waypoint/send_wp_loose')]),
      ('wp_heading', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/control/waypoint/send_wp_heading')]),
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
    waypoint_node,
  ])