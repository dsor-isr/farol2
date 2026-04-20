### Put this file in your personal bringup package ###


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time (passed to sim nodes via farol2_bringup).'
  )

  # Resolve the path to the config package in the install folder
  config_package_path_share = FindPackageShare(LaunchConfiguration('config_package'))

  # Resolve the path to farol2_bringup package in the install folder
  farol_bringup_package_path_share = FindPackageShare('farol2_bringup')

  sim_time_config = PathJoinSubstitution([
    config_package_path_share,
    'config_personal',
    'ros.yaml'
  ])

  ###################
  # Nodes to launch #
  ###################
  farol_bringup_node = Node(
    package='farol2_bringup',
    namespace=[LaunchConfiguration('name'), LaunchConfiguration('id')],
    executable='farol_bringup_node',
    name='farol2_bringup',
    output='screen',
    parameters=[
      sim_time_config,
      {
      'name': LaunchConfiguration('name'),
      'id': LaunchConfiguration('id'),
      'config_package_path_share': config_package_path_share,
      'farol_bringup_package_path_share': farol_bringup_package_path_share,
      # This path will be used to load the file directly in the node,
      # since ROS2 does not provide support for loading dictionaries
      'processes_path': PathJoinSubstitution([
                        config_package_path_share,
                        'config_personal',
                        'vehicles',
                        LaunchConfiguration('name'),
                        'process.yaml',
                        ])
    },
    # Override use_sim_time last so it wins over ros.yaml
    {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    name_arg,
    id_arg,
    config_package_arg,
    use_sim_time_arg,
    # nodes
    farol_bringup_node,
  ])