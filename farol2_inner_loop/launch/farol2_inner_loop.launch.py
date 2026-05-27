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
            {'vehicle_name': LaunchConfiguration('vehicle_name')},
            {'vehicle_id': LaunchConfiguration('vehicle_id')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},

            # load default PID configs
            PathJoinSubstitution([
              FindPackageShare('farol2_bringup'),
              'config',
              LaunchConfiguration('vehicle_name'),
              'default',
              'inner_loop.yaml'
            ]),
            # override config — same path as default when config_to_use == 'default'
          #   PathJoinSubstitution([
          #     FindPackageShare('farol2_bringup'),
          #     'config',
          #     LaunchConfiguration('vehicle_name'),
          #     LaunchConfiguration('config_to_use'),
          #     'inner_loop.yaml'
          #   ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  
  pid_node = Node(
    package='farol2_inner_loop',
    namespace=PathJoinSubstitution([vehicle_ns, 'inner_loop']),
    executable='pid_control',
    name='pid',
    output='screen',
    condition=IfCondition(LaunchConfiguration('pid')),
    parameters=params,
    remappings=[
      # Subscribers
      ('nav_state', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/nav/filter/state')]),
      ('surge_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/surge')]),
      ('sway_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/sway')]),
      ('heave_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/heave')]),
      ('depth_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/depth')]),
      ('altitude_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/altitude')]),
      ('yaw_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/yaw')]),
      ('pitch_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/pitch')]),
      ('roll_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/roll')]),
      ('yaw_rate_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/yaw_rate')]),
      ('pitch_rate_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/pitch_rate')]),
      ('roll_rate_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/roll_rate')]),
      # Publishers
      ('thrust_x', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thrust_x')]),
      ('thrust_y', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thrust_y')]),
      ('thrust_z', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/thrust_z')]),
      ('torque_x', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/torque_x')]),
      ('torque_y', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/torque_y')]),
      ('torque_z', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/torque_z')]),
      ('debug_surge', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/surge/debug')]),
      ('debug_sway', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/sway/debug')]),
      ('debug_heave', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/heave/debug')]),
      ('debug_depth', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/depth/debug')]),
      ('debug_altitude', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/altitude/debug')]),
      ('debug_yaw', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/yaw/debug')]),
      ('debug_pitch', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/pitch/debug')]),
      ('debug_roll', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/roll/debug')]),
      ('debug_yaw_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/yaw_rate/debug')]),
      ('debug_pitch_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/pitch_rate/debug')]),
      ('debug_roll_rate', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/pid/roll_rate/debug')]),
      # Services
      ('change_params', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/change_params')]),
      ('course_control', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/course_control')]),
    ]
  )

  open_loop_node = Node(
    package='farol2_inner_loop',
    namespace=PathJoinSubstitution([vehicle_ns, 'inner_loop']),
    executable='open_loop',
    name='open_loop',
    output='screen',
    condition=IfCondition(LaunchConfiguration('open_loop')),
    parameters=params,
    remappings=[
      # Subscribers
      ('surge_ref', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/inner_loop/ref/surge')]),
      ('mission_status', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/mission_status')]),
      # Publishers
      ('rpm_command', [TextSubstitution(text='/'), vehicle_ns, TextSubstitution(text='/allocation/rpm_command')]),
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
    launch_pid_arg,
    launch_openloop_arg,
    # nodes
    pid_node,
    open_loop_node
  ])
