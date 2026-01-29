from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    ####################
    # Launch arguments #
    ####################
    vehicle_ns_arg = DeclareLaunchArgument(
        'vehicle_ns',
        default_value='vehicle0',
        description='Vehicle namespace'
    )

    config_package_path_real_arg = DeclareLaunchArgument(
        'config_package_path_real',
        default_value='',
        description='Path to config package (src/, real path)'
    )

    ##############################
    # Parameters for sim_clock   #
    ##############################
    params = [
        #sim clock must never use sim time
        {'use_sim_time': False},

        # Default ROS config (contains sim_time.*)
        PathJoinSubstitution([
            LaunchConfiguration('config_package_path_real'),
            'config_personal',
            '.ros_tmp',
            PythonExpression([
                "'default_ros_' + '",
                LaunchConfiguration('vehicle_ns'),
                "' + '.yaml'"
            ])
        ]),

        # Personal ROS config (override)
        PathJoinSubstitution([
            LaunchConfiguration('config_package_path_real'),
            'config_personal',
            '.ros_tmp',
            PythonExpression([
                "'personal_ros_' + '",
                LaunchConfiguration('vehicle_ns'),
                "' + '.yaml'"
            ])
        ]),
    ]

    ###################
    # Nodes to Launch #
    ###################
    sim_clock_node = Node(
        package='sim_time',
        executable='sim_clock_node',  
        name='sim_clock',
        output='screen',
        parameters=params
    )

    return LaunchDescription([
        vehicle_ns_arg,
        config_package_path_real_arg,
        sim_clock_node,
    ])
