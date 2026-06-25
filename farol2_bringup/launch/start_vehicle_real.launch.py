### Put this file in your personal bringup package ###


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from farol2_bringup import get_next_available_vehicle_id


def _launch(context, *args, **kwargs):
  vehicle_name  = LaunchConfiguration('vehicle_name').perform(context)
  vehicle_id    = LaunchConfiguration('vehicle_id').perform(context).strip()
  config_to_use = LaunchConfiguration('config_to_use').perform(context)

  if not vehicle_id:
    vehicle_id = get_next_available_vehicle_id(vehicle_name)

  ####################################################
  # Include drivers launch from drivers own package #
  ####################################################
  drivers = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('farol2_drivers_bringup'),
        'launch',
        f'{vehicle_name}_drivers.launch.py'
      ])
    ]),
    launch_arguments={
      'vehicle_name': vehicle_name,
      'vehicle_id': vehicle_id,
      'vn310': 'false',
      'vn100': 'false',
      'can_thrusters': 'false',
      'can_instrumentation': 'false',
      'airmar200wx': 'false',
      'airmardx900': 'false',
      'ashtech': 'false',
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
          drivers, 
          farol_stack]


def generate_launch_description():

  ####################
  # Launch arguments #
  ####################
  vehicle_name_arg = DeclareLaunchArgument(
    'vehicle_name',
    default_value='myellow',
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
    OpaqueFunction(function=_launch),
  ])

