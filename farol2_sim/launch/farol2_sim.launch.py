from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from farol2_sim.simulation_launch_utils import configure_domain, wait_for_clock


def _start_simulator(context, node):
  wait_for_clock(context.environment['ROS_DOMAIN_ID'], LaunchConfiguration('clock_timeout').perform(context))
  return [node]


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

  ##############################################################
  # Parameters — vehicle sim nodes are driven by the /clock pub #
  ##############################################################
  params = [
    {'vehicle_name': LaunchConfiguration('vehicle_name')},
    {'vehicle_id': LaunchConfiguration('vehicle_id')},
    {'frame_prefix': [vehicle_ns, TextSubstitution(text='/')]},

    # default sim config
    PathJoinSubstitution([
      FindPackageShare('farol2_bringup'),
      'config',
      LaunchConfiguration('vehicle_name'),
      'default',
      'sim.yaml'
    ]),
    # Selected config overlays the default (same file for config_to_use=default).
    PathJoinSubstitution([
      FindPackageShare('farol2_bringup'), 'config',
      LaunchConfiguration('vehicle_name'), LaunchConfiguration('config_to_use'), 'sim.yaml'
    ]),

    # Shared world environment takes precedence over any old per-vehicle values.
    LaunchConfiguration('world_config'),

    {'use_sim_time': True},
  ]

  ###################
  # Nodes to launch #
  ###################
  
  vehicle_sim_node = Node(
    package='farol2_sim',
    namespace=[vehicle_ns, '/sim'],
    executable='vehicle_sim',
    name='vehicle_sim',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('rudder_cmd', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/rudder_cmd')]),
      ('rpm_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rpm_command')]),
      ('control_surface_angle_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/control_surface_angle')]),
      # Publishers
      ('position', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/position')]),
      ('body_velocity', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/body_velocity')]),
      ('orientation', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/orientation')]),
      ('orientation_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/orientation_rate')]),
      ('body_acceleration', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/body_acceleration')]),
      ('angular_acceleration', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/sim/angular_acceleration')]),
      ('joint_states', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/joint_states')]),
      ('control_surface_angle', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/control_surface_angle')]),
      ('imu', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/imu')]),
      ('gnss', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/gnss')]),
      ('ned_utm', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/ned_utm')]),
      ('velocity_over_ground', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/velocity_over_ground')]),
      ('velocity_through_water', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/velocity_through_water')]),
      ('depth', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/depth')]),
      ('altimeter', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/altimeter')]),
      ('thruster_rpm', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/measurement/thruster_rpm')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    vehicle_id_arg,
    vehicle_name_arg,
    config_to_use,
    DeclareLaunchArgument('world_config',
      default_value=PathJoinSubstitution([
        FindPackageShare('farol2_bringup'), 'config', 'simple_world.yaml']),
      description='Shared world YAML; use the same file as the world launcher.'),
    DeclareLaunchArgument('ros_domain_id',
      default_value='0',
      description='DDS domain of the world to join.'),
    DeclareLaunchArgument('clock_timeout', default_value='5.0',
      description='Wall seconds to wait for advancing /clock.'),
    OpaqueFunction(function=configure_domain),
    OpaqueFunction(function=_start_simulator, kwargs={'node': vehicle_sim_node}),
  ])
