from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  vehicle_name_arg = DeclareLaunchArgument(
    'vehicle_name',
    default_value='vehicle',
    description='Vehicle name'
  )

  vehicle_id_arg = DeclareLaunchArgument(
    'vehicle_id',
    default_value='0',
    description='Vehicle ID'
  )

  config_to_use_arg = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config folder to use.'
  )

  vehicle_ns = PythonExpression(["'", LaunchConfiguration('vehicle_name'), "' + '", LaunchConfiguration('vehicle_id'), "'"])

  sim_config = PathJoinSubstitution([
    FindPackageShare('farol2_bringup'),
    'config',
    LaunchConfiguration('vehicle_name'),
    LaunchConfiguration('config_to_use'),
    'sim.yaml'
  ])

  clock_node = Node(
    package='farol2_sim',
    namespace=[vehicle_ns, '/sim'],
    executable='sim_clock',
    name='sim_clock',
    output='screen',
    parameters=[
      sim_config,
      {'use_sim_time': False},
    ],
  )

  return LaunchDescription([
    vehicle_name_arg,
    vehicle_id_arg,
    config_to_use_arg,
    clock_node,
  ])
