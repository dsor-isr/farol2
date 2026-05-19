import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

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
    description='Path to the config package, usually the personal bringup of the workspace, in the install folder'
  )

  config_package_path_real_arg = DeclareLaunchArgument(
    'config_package_path_real',
    default_value='',
    description='Path to the config package, usually the personal bringup of the workspace, in the src folder'
  )

  launch_compression_arg = DeclareLaunchArgument(
    'compression',
    default_value='false',
    description='Boolean to determine if compression is used in bag recording.'
  )

  launch_prefix_arg = DeclareLaunchArgument(
    'prefix',
    default_value=LaunchConfiguration('vehicle_ns'),
    description='Boolean to determine if compression is used in bag recording.'
  )
  
  #############
  # Processes #
  #############
  timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S") # get timestamp

  # NO COMPRESSION
  bag_record_process = ExecuteProcess(
    cmd=['ros2', 'bag', 'record', # record bag
         '-a', # record all topics
         '-o', PythonExpression(["'", LaunchConfiguration('prefix'), "' + ", f"'_{timestamp}'"]),
         '--log-level', 'warn'], # set log level to WARN to avoid unnecessary INFO-level logs on stdout
    cwd=os.environ.get('ROSDATA_DIR'),
    name='bag_record_process',
    output='log',
    condition=UnlessCondition(LaunchConfiguration('compression')),
  )

  # WITH COMPRESSION
  bag_record_process_with_compression = ExecuteProcess(
    cmd=['ros2', 'bag', 'record', # record bag
         '-a', # record all topics
         '-o', PythonExpression(["'", LaunchConfiguration('prefix'), "' + ", f"'_{timestamp}'"]),
         '--log-level', 'warn', # set log level to WARN to avoid unnecessary INFO-level logs on stdout
         '--compression-mode', 'file', '--compression-format', 'zstd'], # compression parameters
    cwd=os.environ.get('ROSDATA_DIR'),
    name='bag_record_process',
    output='log',
    condition=IfCondition(LaunchConfiguration('compression')),
  )

  ##########################################################
  # Return launch description with arguments and processes #
  ##########################################################
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    vehicle_name_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    launch_compression_arg,
    launch_prefix_arg,
    # processes
    bag_record_process,
    bag_record_process_with_compression,
  ])