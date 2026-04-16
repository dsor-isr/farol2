### Put this file in your personal bringup package ###


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  ####################
  # Launch arguments #
  ####################
  name_arg = DeclareLaunchArgument(
    'name',
    default_value='magicelectric',
    description='Name of the vehicle to be launched.'
  )

  id_arg = DeclareLaunchArgument(
    'id',
    default_value='0',
    description='ID of the vehicle to be launched.'
  )

  config_package_arg = DeclareLaunchArgument(
    'config_package',
    default_value='personal_bringup',
    description='Package where the configuration files are.'
  )

  ###############################################################
  # Include the standard vehicle stack with use_sim_time forced #
  ###############################################################
  farol_stack = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('personal_bringup'), 'launch', 'start_vehicle.launch.py'])
    ]),
    launch_arguments={
      'name': LaunchConfiguration('name'),
      'id': LaunchConfiguration('id'),
      'config_package': LaunchConfiguration('config_package'),
      'use_sim_time': 'true',
    }.items()
  )

  return LaunchDescription([
    name_arg,
    id_arg,
    config_package_arg,
    farol_stack,
  ])

