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

  config_to_use = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config folder to use.'
  )

  vehicle_ns = PythonExpression(["'", LaunchConfiguration('vehicle_name'), "' + '", LaunchConfiguration('vehicle_id'), "'"])

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

  ##############################################################
  # Parameters — vehicle sim nodes are driven by the /clock pub #
  ##############################################################
  params = [
    {'vehicle_name': LaunchConfiguration('vehicle_name')},
    {'vehicle_id': LaunchConfiguration('vehicle_id')},

    # default sim config
    PathJoinSubstitution([
      FindPackageShare('farol2_bringup'),
      'config',
      LaunchConfiguration('vehicle_name'),
      'default',
      'sim.yaml'
    ]),
    # override config — same path as default when config_to_use == 'default'
    # PathJoinSubstitution([
    #   FindPackageShare('farol2_bringup'),
    #   'config',
    #   LaunchConfiguration('vehicle_name'),
    #   LaunchConfiguration('config_to_use'),
    #   'sim.yaml'
    # ]),

    {'use_sim_time': True},
  ]

  ###################
  # Nodes to launch #
  ###################
  
  magic_electric_sim_node = Node(
    package='farol2_sim',
    namespace=[vehicle_ns, '/sim'],
    executable='magic_electric_sim',
    name='magic_electric_sim',
    output='screen',
    condition=IfCondition(LaunchConfiguration('magic_electric_sim')),
    parameters=params,
    remappings=[
      # Subscribers
      ('rpm_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rpm_command')]),
      ('rudder_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rudder_command')]),
      # Publishers
      ('position', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/position')]),
      ('body_velocity', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/body_velocity')]),
      ('orientation', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/orientation')]),
      ('orientation_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/orientation_rate')]),
      ('body_acceleration', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/body_acceleration')]),
      ('angular_acceleration', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/angular_acceleration')]),
      ('rudder_angle', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/rudder_angle')]),
      ('imu', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/imu')]),
      ('gnss', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/gnss')]),
      ('ned_utm', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/ned_utm')]),
      ('velocity_over_ground', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/velocity_over_ground')]),
      ('velocity_through_water', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/velocity_through_water')]),
      ('depth', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/depth')]),
    ]
  )

  auv_sim_node = Node(
    package='farol2_sim',
    namespace=[vehicle_ns, '/sim'],
    executable='auv_sim',
    name='auv_sim',
    output='screen',
    condition=IfCondition(LaunchConfiguration('auv_sim')),
    parameters=params,
    remappings=[
      # Subscribers
      ('rpm_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rpm_command')]),
      # Publishers
      ('position', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/position')]),
      ('body_velocity', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/body_velocity')]),
      ('orientation', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/orientation')]),
      ('orientation_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/orientation_rate')]),
      ('body_acceleration', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/body_acceleration')]),
      ('angular_acceleration', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/angular_acceleration')]),
      ('imu', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/imu')]),
      ('gnss', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/gnss')]),
      ('ned_utm', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/ned_utm')]),
      ('velocity_over_ground', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/velocity_over_ground')]),
      ('velocity_through_water', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/velocity_through_water')]),
      ('depth', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/depth')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    vehicle_id_arg,
    vehicle_name_arg,
    config_to_use,
    launch_magic_electric_sim_arg,
    launch_auv_sim_arg,
    magic_electric_sim_node,
    auv_sim_node,
  ])
