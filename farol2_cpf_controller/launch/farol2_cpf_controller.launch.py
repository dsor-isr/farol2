from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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
      description='Vehicle namespace'
    )
    
    vehicle_name_arg = DeclareLaunchArgument(
      'vehicle_name', 
      default_value='vehicle', 
      description='Vehicle name'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
      'use_sim_time', default_value='false', 
      description='Use sim time'
    )
    
    config_package_path_share_arg = DeclareLaunchArgument(
      'config_package_path_share', default_value='', 
      description='Personal config share path (install)'
    )

    run_cpf_arg = DeclareLaunchArgument(
      'cpf', 
      default_value='true', 
      description='Run CPF node'
    )

    ###################################
    # Define parameters for all nodes #
    ###################################
    params = [
        
              {'vehicle_ns': LaunchConfiguration('vehicle_ns')},
              {'use_sim_time': LaunchConfiguration('use_sim_time')},

              PathJoinSubstitution([
                  FindPackageShare('farol2_bringup'),
                  'config_default',
                  'vehicles',
                  LaunchConfiguration('vehicle_name'),
                  'cpf_controller.yaml'
              ]),

              PathJoinSubstitution([
                  LaunchConfiguration('config_package_path_share'),
                  'config_personal',
                  'vehicles',
                  LaunchConfiguration('vehicle_name'),
                  'cpf_controller.yaml'
              ])
    ]

    cpf_node = Node(
        package='farol2_cpf_controller',
        namespace=PathJoinSubstitution([LaunchConfiguration('vehicle_ns'), 'control']),
        executable='cpf_controller_node',
        name='cpf_controller',
        output='screen',
        parameters=params,
        condition=IfCondition(LaunchConfiguration('cpf')),
        remappings=[
            # topics used in the design/images -- adapt names if your node uses others
            ('external_gamma', [TextSubstitution(text='/cpf_broadcast/path_data')]),
            ('internal_gamma', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/gamma')]),
            # Publishers 
            ('vc', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/path_following/vc')]),
            ('broadcast_data', [TextSubstitution(text='/cpf_broadcast/path_data')]),
            # services (ChangeTopology / StartStop)
            ('change_topology', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/cpf_controller/change_topology')]),
            ('start_cpf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/cpf_controller/start_cpf')]),
            ('stop_cpf', [TextSubstitution(text='/'), LaunchConfiguration('vehicle_ns'), TextSubstitution(text='/control/cpf_controller/stop_cpf')]),
        ]
    )

    return LaunchDescription([
        vehicle_ns_arg,
        vehicle_name_arg,
        use_sim_time_arg,
        config_package_path_share_arg,
        run_cpf_arg,
        cpf_node
    ])