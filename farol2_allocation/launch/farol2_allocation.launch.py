from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
            {'frame_prefix': [vehicle_ns, TextSubstitution(text='/')]},
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

  thruster_allocation_node = Node(
    package='farol2_allocation',
    namespace=PathJoinSubstitution([vehicle_ns, 'allocation']),
    executable='allocation_node',
    name='thruster_allocation',
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
      ('nav_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      ('mission_status', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/mission_status')]),
      # Publishers
      ('rpm_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rpm_command')]),
      ('rudder_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rudder_command')]),
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
    # nodes
    thruster_allocation_node,
  ])
