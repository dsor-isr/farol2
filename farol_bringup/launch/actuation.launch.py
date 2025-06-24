from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

  ###################################
  # Define parameters for all nodes #
  ###################################
  params = [
            # vehicle namespace
            {'vehicle_ns': LaunchConfiguration('vehicle_ns')},

            # load default ROS configurations (from tmp files)
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_real'),
              'config_personal',
              '.ros_tmp',
              PythonExpression(["'default_ros_' + '", LaunchConfiguration('vehicle_ns'), "' + '.yaml'"])
            ]),

            # override default with personal ROS configurations (from tmp files)
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_real'),
              'config_personal',
              '.ros_tmp',
              PythonExpression(["'personal_ros_' + '", LaunchConfiguration('vehicle_ns'), "' + '.yaml'"])
            ]),

            # load default actuation configs
            PathJoinSubstitution([
              FindPackageShare('farol_bringup'),
              'config_default',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'actuation.yaml'
            ]),
            
            # override with personal actuation configs
            PathJoinSubstitution([
              LaunchConfiguration('config_package_path_share'),
              'config_personal',
              'vehicles',
              LaunchConfiguration('vehicle_name'),
              'actuation.yaml'
            ]),
          ]


  ###################
  # Nodes to launch #
  ###################
  static_thruster_allocation_node = Node(
    package='control_allocation',
    namespace=[LaunchConfiguration('vehicle_ns'), '/actuation'],
    executable='static_thruster_allocation',
    name='static_thruster_allocation',
    output='screen',
    parameters=params
  )

  ######################################################
  # Return launch description with arguments and nodes #
  ######################################################
  return LaunchDescription([
    # launch arguments
    vehicle_ns_arg,
    vehicle_name_arg,
    config_package_path_share_arg,
    config_package_path_real_arg,
    # nodes
    static_thruster_allocation_node,
  ])