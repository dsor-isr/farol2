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

  launch_pid_arg = DeclareLaunchArgument(
    'pid',
    default_value='true',
    description='Boolean to determine if "pid" node is launched.'
  )

  launch_openloop_arg = DeclareLaunchArgument(
  'open_loop',
  default_value='true',
  description='Boolean to determine if "open_loop" node is launched.'
  )

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            # load default PID configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'inner_loop.yaml'
            ]),
            
            # override with personal PID configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'inner_loop.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  
  pid_node = Node(
    package='farol2_inner_loop',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'inner_loop']),
    executable='pid_control',
    name='pid',
    output='screen',
    condition=IfCondition(LaunchConfiguration('pid')),
    parameters=params,
    remappings=[
      # Subscribers
      ('nav_state', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/nav/filter/state')]),
      ('surge_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/surge')]),
      ('sway_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/sway')]),
      ('heave_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/heave')]),
      ('depth_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/depth')]),
      ('altitude_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/altitude')]),
      ('yaw_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/yaw')]),
      ('pitch_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/pitch')]),
      ('roll_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/roll')]),
      ('yaw_rate_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/yaw_rate')]),
      ('pitch_rate_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/pitch_rate')]),
      ('roll_rate_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/roll_rate')]),
      # Publishers
      ('thrust_x', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thrust_x')]),
      ('thrust_y', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thrust_y')]),
      ('thrust_z', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/thrust_z')]),
      ('torque_x', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/torque_x')]),
      ('torque_y', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/torque_y')]),
      ('torque_z', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/torque_z')]),
      ('debug_surge', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/surge/debug')]),
      ('debug_sway', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/sway/debug')]),
      ('debug_heave', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/heave/debug')]),
      ('debug_depth', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/depth/debug')]),
      ('debug_altitude', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/altitude/debug')]),
      ('debug_yaw', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/yaw/debug')]),
      ('debug_pitch', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/pitch/debug')]),
      ('debug_roll', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/roll/debug')]),
      ('debug_yaw_rate', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/yaw_rate/debug')]),
      ('debug_pitch_rate', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/pitch_rate/debug')]),
      ('debug_roll_rate', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/pid/roll_rate/debug')]),
      # Services
      ('change_params', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/change_params')]),
      ('course_control', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/course_control')]),
    ]
  )

  open_loop_node = Node(
    package='farol2_inner_loop',
    namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'inner_loop']),
    executable='open_loop',
    name='open_loop',
    output='screen',
    condition=IfCondition(LaunchConfiguration('open_loop')),
    parameters=params,
    remappings=[
      # Subscribers
      ('surge_ref', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/inner_loop/ref/surge')]),
      ('mission_status', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/mission_status')]),
      # Publishers
      ('rpm_command', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/allocation/rpm_command')]),
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
    launch_pid_arg,
    launch_openloop_arg,
    # nodes
    pid_node,
    open_loop_node
  ])
