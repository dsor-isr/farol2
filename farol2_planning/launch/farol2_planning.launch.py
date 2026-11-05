from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  
  ####################
  # Launch arguments #
  ####################
  vehicle_id_arg = DeclareLaunchArgument(
    'vehicle_id',
    default_value='0',
    description='Vehicle ID'
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

  config_to_use = DeclareLaunchArgument(
    'config_to_use',
    default_value='default',
    description='Config folder to use.'
  )

  vehicle_ns = PythonExpression(["'", LaunchConfiguration('vehicle_name'), "' + '", LaunchConfiguration('vehicle_id'), "'"])

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_name': LaunchConfiguration('vehicle_name')},
            {'vehicle_id': LaunchConfiguration('vehicle_id')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            # load default planning configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config',
              LaunchConfiguration('vehicle_name'),
              'default',
              'planning.yaml'
            ]),
            # override config — same path as default when config_to_use == 'default'
            # PathJoinSubstitution([
            #   FindPackageShare('farol2_bringup'),
            #   'config',
            #   LaunchConfiguration('vehicle_name'),
            #   LaunchConfiguration('config_to_use'),
            #   'planning.yaml'
            # ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  paths_node = Node(
    package='farol2_planning',
    namespace=PathJoinSubstitution([vehicle_ns, 'planning']),
    executable='path_node',
    name='paths',
    output='screen',
    parameters=params,
    remappings=[
      # Subscribers
      ('gamma', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/gamma')]),
      ('vehicle_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      # Publishers
      ('path_data', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/path_data')]),
      ('virtual_target_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/path_following/virtual_state')]),
      # Services
      ('reset_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/ResetPath')]),
      ('set_mode', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SetMode')]),
      ('arc2d_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SpawnArc2DPath')]),
      ('bernoulli_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SpawnBernoulliPath')]),
      ('circle2d_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SpawnCircle2DPath')]),
      ('line_path', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SpawnLinePath')]),
      ('speed_const_rabbit_speed', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SetConstVdRabbit')]),
      ('speed_const_vehicle_speed', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/SetConstVdVehicle')]),
    ]
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_id_arg,
    vehicle_name_arg,
    use_sim_time_arg,
    config_to_use,
    # nodes
    paths_node,
  ])