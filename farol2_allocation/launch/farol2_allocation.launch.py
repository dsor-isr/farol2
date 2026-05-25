from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


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

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
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

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            # load default allocation configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'allocation.yaml'
            ]),
            
            # override with personal allocation configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'allocation.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  




  thruster_allocation_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'allocation']),
    executable='thruster_allocation',
    name='thruster_allocation',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('body_wrench_request', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/body_wrench_request')]),
      ('nav_state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ('mission_status', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/mission_status')]),
      # Publishers
      ('thruster_force', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thruster_force')]),
      ('rudder_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rudder_command')]),
    ]
  )

  rpm_conversion_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'allocation']),
    executable='rpm_conversion',
    name='rpm_conversion',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('thruster_force', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thruster_force')]),
      ('nav_state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
    ]
  )

  throttle_conversion_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'allocation']),
    executable='throttle_conversion',
    name='throttle_conversion',
    output='screen',
    condition=IfCondition(LaunchConfiguration('throttle_conversion')),
    parameters=params,
    remappings=[
      # Subscribers
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
      # Publishers
      ('throttle_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/throttle_command')]),
    ]
  )

  wrench_manager_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'allocation']),
    executable='wrench_manager',
    name='wrench_manager',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('thrust_x', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thrust_x')]),
      ('thrust_y', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thrust_y')]),
      ('thrust_z', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thrust_z')]),
      ('torque_x', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/torque_x')]),
      ('torque_y', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/torque_y')]),
      ('torque_z', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/torque_z')]),
      # Publishers
      ('body_wrench_request', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/body_wrench_request')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    vehicle_name_arg,
    use_sim_time_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    # nodes
    thruster_allocation_node,
    rpm_conversion_node,
    wrench_manager_node,
  ])
