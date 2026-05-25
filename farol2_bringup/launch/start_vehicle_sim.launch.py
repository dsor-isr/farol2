from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

  config_package_path_real_arg = DeclareLaunchArgument(
    'config_package_path_real',
    default_value=PathJoinSubstitution([
      FindPackageShare(LaunchConfiguration('config_package')),
      '..', '..', '..', '..',
      'src',
      LaunchConfiguration('config_package')
    ]),
    description='Path to the config package in the src folder.'
  )

  vehicle_ns = PythonExpression([
    "'", LaunchConfiguration('name'), "' + '", LaunchConfiguration('id'), "'"
  ])

  is_magic_electric = PythonExpression([
    "'", LaunchConfiguration('name'), "' == 'magicelectric'"
  ])

  is_not_magic_electric = PythonExpression([
    "'", LaunchConfiguration('name'), "' != 'magicelectric'"
  ])

  ###################################################
  # Include simulation launch (before Farol stack) #
  ###################################################
  simulation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('farol2_sim'),
        'launch',
        'farol2_sim.launch.py'
      ])
    ]),
    launch_arguments={
      'vehicle_ns': vehicle_ns,
      'vehicle_name': LaunchConfiguration('name'),
      'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
      'config_package_path_real': LaunchConfiguration('config_package_path_real'),
      'magic_electric_sim': is_magic_electric,
      'auv_sim': is_not_magic_electric,
    }.items()
  )

  ###############################################################
  # Include the standard vehicle stack with use_sim_time forced #
  ###############################################################
  farol_stack = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([FindPackageShare('personal_bringup'), 'launch', 'start_farol2.launch.py'])
    ]),
    launch_arguments={
      'name': LaunchConfiguration('name'),
      'id': LaunchConfiguration('id'),
      'config_package': LaunchConfiguration('config_package'),
      'config_package_path_real': LaunchConfiguration('config_package_path_real'),
      'use_sim_time': 'true',
    }.items()
  )

  return LaunchDescription([
    name_arg,
    id_arg,
    config_package_arg,
    config_package_path_real_arg,
    simulation,
    farol_stack,
  ])
