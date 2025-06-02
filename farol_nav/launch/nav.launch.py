from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  
  # Launch arguments
  vehicle_ns_arg = DeclareLaunchArgument(
    'vehicle_ns',
    default_value='vehicle0',
    description='Vehicle namespace for topics, nodes, etc.'
  )

  # Nodes to launch
  navigation_filter_node = Node (
    package='farol_nav',
    namespace=LaunchConfiguration('vehicle_ns'),
    executable='navigation_filter_node',
    name='navigation_filter'
  )

  # Return launch description with arguments and nodes
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    # nodes
    navigation_filter_node,
  ])