from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from farol2_bringup import get_next_available_vehicle_id
from farol2_sim.simulation_launch_utils import configure_domain, wait_for_clock


def _launch(context, *args, **kwargs):
  domain = int(context.environment['ROS_DOMAIN_ID'])
  wait_for_clock(domain, LaunchConfiguration('clock_timeout').perform(context))
  vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
  vehicle_id   = LaunchConfiguration('vehicle_id').perform(context).strip()
  config_to_use = LaunchConfiguration('config_to_use').perform(context)

  if not vehicle_id:
    vehicle_id = get_next_available_vehicle_id(vehicle_name, domain_id=domain)

  ###################################################
  # Include simulation launch (before Farol stack) #
  ###################################################
  simulation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('farol2_sim'), 'launch', 'farol2_sim.launch.py'])
    ]),
    launch_arguments={
      'vehicle_name': vehicle_name,
      'vehicle_id': vehicle_id,
      'config_to_use': config_to_use,
      'ros_domain_id': str(domain),
      'clock_timeout': LaunchConfiguration('clock_timeout'),
      'world_config': LaunchConfiguration('world_config'),
    }.items()
  )

  ###############################################################
  # Include the standard vehicle stack with use_sim_time forced #
  ###############################################################
  farol_stack = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('farol2_bringup'), 'launch', 'start_farol2.launch.py'])
    ]),
    launch_arguments={
      'vehicle_name': vehicle_name,
      'vehicle_id': vehicle_id,
      'config_to_use': config_to_use,
      'use_sim_time': 'true',
    }.items()
  )

  # Keep vehicle_id in context so ros2 launch tooling can report it.
  return [SetLaunchConfiguration('vehicle_id', vehicle_id),
          simulation,
          farol_stack]


def generate_launch_description():

  ####################
  # Launch arguments #
  ####################
  vehicle_name_arg = DeclareLaunchArgument(
    'vehicle_name',
    default_value='magicelectric',
    description='Name of the vehicle to be launched.'
  )

  vehicle_id_arg = DeclareLaunchArgument(
    'vehicle_id',
    default_value='',
    description='ID of the vehicle to be launched. Leave empty to auto-assign the next available id.'
  )

  config_to_use_arg = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config to use.'
  )

  return LaunchDescription([
    vehicle_name_arg,
    vehicle_id_arg,
    config_to_use_arg,
    DeclareLaunchArgument('world_config',
      default_value=PathJoinSubstitution([
        FindPackageShare('farol2_bringup'), 'config', 'simple_world.yaml']),
      description='Shared world YAML; use the same file as the world launcher.'),
    DeclareLaunchArgument('ros_domain_id',
      default_value='0',
      description='DDS domain of the world to join.'),
    DeclareLaunchArgument('clock_timeout', default_value='5.0',
      description='Wall seconds to wait for advancing /clock before starting the stack.'),
    OpaqueFunction(function=configure_domain),
    OpaqueFunction(function=_launch),
  ])
