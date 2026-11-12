"""Start one independent simulation world (currently its clock only)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from farol2_sim.simulation_launch_utils import configure_domain, ensure_clock_unowned


def _start_world(context):
    ensure_clock_unowned(context.environment['ROS_DOMAIN_ID'])
    return [Node(
        package='farol2_sim', executable='sim_clock', name='sim_clock',
        namespace='world', output='screen',
        parameters=[LaunchConfiguration('world_config'), {'use_sim_time': False}],
        on_exit=Shutdown(reason='World clock exited'),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id', default_value='0',
            description='DDS domain for this world instance (0..232); defaults to 0.'),
        DeclareLaunchArgument(
            'world_config', default_value=PathJoinSubstitution([
                FindPackageShare('farol2_bringup'), 'config', 'simple_world.yaml']),
            description='World configuration YAML, independent of vehicle settings.'),
        OpaqueFunction(function=configure_domain),
        OpaqueFunction(function=_start_world),
    ])
