from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from farol2_bringup import get_next_available_vehicle_id


def _launch(context, *args, **kwargs):
  vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
  vehicle_id   = LaunchConfiguration('vehicle_id').perform(context).strip()
  config_to_use = LaunchConfiguration('config_to_use').perform(context)

  if not vehicle_id:
    vehicle_id = get_next_available_vehicle_id(vehicle_name)

  use_magic_electric_sim = vehicle_name == 'magicelectric'

  ###################################################
  # Include simulation launch (before Farol stack) #
  ###################################################
  clock = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('farol2_sim'), 'launch', 'sim_clock.launch.py'])
    ]),
    launch_arguments={
      'vehicle_name': vehicle_name,
      'vehicle_id': vehicle_id,
      'config_to_use': config_to_use,
    }.items(),
    condition=IfCondition(LaunchConfiguration('farol2_clock'))
  )

  simulation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('farol2_sim'), 'launch', 'farol2_sim.launch.py'])
    ]),
    launch_arguments={
      'vehicle_name': vehicle_name,
      'vehicle_id': vehicle_id,
      'config_to_use': config_to_use,
      'magic_electric_sim': 'true' if use_magic_electric_sim else 'false',
      'auv_sim': 'false' if use_magic_electric_sim else 'true',
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
      'use_sim_time': 'false',
    }.items()
  )

  # Keep vehicle_id in context so ros2 launch tooling can report it.
  return [SetLaunchConfiguration('vehicle_id', vehicle_id),
          clock,
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

  farol2_clock_arg = DeclareLaunchArgument(
    'farol2_clock',
    default_value='true',
    description='Launch the Farol2 simulation clock.'
  )

  return LaunchDescription([
    vehicle_name_arg,
    vehicle_id_arg,
    config_to_use_arg,
    farol2_clock_arg,
    OpaqueFunction(function=_launch),
  ])
