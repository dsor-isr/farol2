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

  config_package_path_share_arg = DeclareLaunchArgument(
    'config_package_path_share',
    default_value='',
    description='Path to the config package in the install folder'
  )

  config_package_path_real_arg = DeclareLaunchArgument(
    'config_package_path_real',
    default_value='',
    description='Path to the config package in the src folder'
  )

  launch_magic_electric_sim_arg = DeclareLaunchArgument(
    'magic_electric_sim',
    default_value='false',
    description='Launch the magic_electric_sim node.'
  )

  launch_auv_sim_arg = DeclareLaunchArgument(
    'auv_sim',
    default_value='false',
    description='Launch the auv_sim node.'
  )

  ############################################################
  # Parameters — vehicle sim nodes always use wall-clock time #
  ############################################################
  params = [
    {'vehicle_ns': LaunchConfiguration('vehicle_ns')},

    # default sim config
    PathJoinSubstitution([
      FindPackageShare('farol2_bringup'),
      'config_default',
      'vehicles',
      LaunchConfiguration('vehicle_name'),
      'sim.yaml'
    ]),

    # personal sim config override
    PathJoinSubstitution([
      LaunchConfiguration('config_package_path_share'),
      'config_personal',
      'vehicles',
      LaunchConfiguration('vehicle_name'),
      'sim.yaml'
    ]),

    # vehicle sim nodes are the /clock authority — they must NEVER use sim time
    {'use_sim_time': False},
  ]

  ###################
  # Nodes to launch #
  ###################
  
  magic_electric_sim_node = Node(
    package='farol2_sim',
    namespace=[LaunchConfiguration('vehicle_ns'), '/sim'],
    executable='magic_electric_sim',
    name='magic_electric_sim',
    output='screen',
    condition=IfCondition(LaunchConfiguration('magic_electric_sim')),
    parameters=params,
    remappings=[
      # Subscribers
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
      ('rudder_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rudder_command')]),
      # Publishers
      ('position', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/position')]),
      ('body_velocity', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/body_velocity')]),
      ('orientation', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/orientation')]),
      ('orientation_rate', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/orientation_rate')]),
      ('body_acceleration', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/body_acceleration')]),
      ('angular_acceleration', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/angular_acceleration')]),
      ('rudder_angle', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/rudder_angle')]),
      ('measurement', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement')]),
    ]
  )

  auv_sim_node = Node(
    package='farol2_sim',
    namespace=[LaunchConfiguration('vehicle_ns'), '/sim'],
    executable='auv_sim',
    name='auv_sim',
    output='screen',
    condition=IfCondition(LaunchConfiguration('auv_sim')),
    parameters=params,
    remappings=[
      # Subscribers
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
      # Publishers
      ('position', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/position')]),
      ('body_velocity', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/body_velocity')]),
      ('orientation', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/orientation')]),
      ('orientation_rate', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/orientation_rate')]),
      ('body_acceleration', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/body_acceleration')]),
      ('angular_acceleration', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/sim/angular_acceleration')]),
      ('measurement', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/measurement')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    vehicle_ns_arg,
    vehicle_name_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    launch_magic_electric_sim_arg,
    launch_auv_sim_arg,
    magic_electric_sim_node,
    auv_sim_node,
  ])
