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
  vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name', default_value='magicelectric', description='Vehicle name.'
    )
  vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id', default_value='0', description='Vehicle ID.'
    )

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
  )

  config_to_use_arg = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config folder to use.'
  )
  ## TO DIE BELLOW
  launch_static_thruster_allocation_arg = DeclareLaunchArgument(
    'static_thruster_allocation',
    default_value='true',
    description='Boolean to determine if "static_thruster_allocation" node is launched.'
  )

  launch_thruster_rudder_allocation_arg = DeclareLaunchArgument(
    'thruster_rudder_allocation',
    default_value='false',
    description='Boolean to determine if "thruster_rudder_allocation" node is launched.'
  )

  launch_throttle_conversion_arg = DeclareLaunchArgument(
    'throttle_conversion',
    default_value='false',
    description='Boolean to determine if "throttle_conversion" node is launched.'
  )
  #### TO DIE ABOVE
  
  ############################
  # Build vehicle namespace #
  ############################
  vehicle_ns = PythonExpression(["'", LaunchConfiguration('vehicle_name'), "' + '", LaunchConfiguration('vehicle_id'), "'"])


  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_name': LaunchConfiguration('vehicle_name')},
            {'vehicle_id': LaunchConfiguration('vehicle_id')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            # load default allocation configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config',
              LaunchConfiguration('vehicle_name'),
              "default",
              'allocation.yaml'
            ]),
            # override config — same path as default when config_to_use == 'default'
            # PathJoinSubstitution([
            #   FindPackageShare('farol2_bringup'),
            #   'config',
            #   LaunchConfiguration('vehicle_name'),
            #   LaunchConfiguration('config_to_use'),
            #   'allocation.yaml'
            # ]),
          ]


  ###################
  # Nodes to launch #
  ###################

  static_thruster_allocation_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([vehicle_ns, 'allocation']),
    executable='static_thruster_allocation',
    name='static_thruster_allocation',
    output='screen',
    condition=IfCondition(LaunchConfiguration('static_thruster_allocation')),
    parameters=params,
    remappings=[
      # Subscribers
      ('body_wrench_request', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/body_wrench_request')]),
      # Publishers
      ('thruster_force', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thruster_force')]),
    ]
  )

  thruster_rudder_allocation_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([vehicle_ns, 'allocation']),
    executable='thruster_rudder_allocation',
    name='thruster_rudder_allocation',
    output='screen',
    condition=IfCondition(LaunchConfiguration('thruster_rudder_allocation')),
    parameters=params,
    remappings=[
      # Subscribers
      ('body_wrench_request', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/body_wrench_request')]),
      ('nav_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      ('mission_status', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/mission_status')]),
      # Publishers
      ('thruster_force', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thruster_force')]),
      ('rudder_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rudder_command')]),
    ]
  )

  rpm_conversion_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([vehicle_ns, 'allocation']),
    executable='rpm_conversion',
    name='rpm_conversion',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('thruster_force', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thruster_force')]),
      ('nav_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('rpm_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rpm_command')]),
    ]
  )

  wrench_manager_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([vehicle_ns, 'allocation']),
    executable='wrench_manager',
    name='wrench_manager',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('thrust_x', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thrust_x')]),
      ('thrust_y', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thrust_y')]),
      ('thrust_z', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thrust_z')]),
      ('torque_x', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/torque_x')]),
      ('torque_y', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/torque_y')]),
      ('torque_z', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/torque_z')]),
      # Publishers
      ('body_wrench_request', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/body_wrench_request')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_name_arg,
    vehicle_id_arg,
    use_sim_time_arg,
    config_to_use_arg,
    launch_static_thruster_allocation_arg,
    launch_thruster_rudder_allocation_arg,
    launch_throttle_conversion_arg,
    # nodes
    static_thruster_allocation_node,
    thruster_rudder_allocation_node,
    rpm_conversion_node,
    wrench_manager_node,
  ])