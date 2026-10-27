from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_name_arg = DeclareLaunchArgument(
        "vehicle_name",
        default_value="myellow0",
        description="Vehicle name used by the distributed planner",
    )

    config_to_use_arg = DeclareLaunchArgument(
        "config_to_use",
        default_value="default",
        description="Config folder to use",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    config_file = PathJoinSubstitution([
        FindPackageShare("farol2_motion_planning"),
        "config",
        "config.yaml",
    ])

    prefix_arg = DeclareLaunchArgument("prefix", default_value="", description="Debugger prefix")

    distributed_planner = Node(
        package="farol2_motion_planning",
        namespace="farol2_motion_planning/distributed_planner_node",
        executable="distributed_planner_node",
        name=LaunchConfiguration("vehicle_name"),
        prefix=LaunchConfiguration("prefix"),
        output="screen",
        parameters=[
            config_file,
            {
                "vehicle_name": LaunchConfiguration("vehicle_name"),
                "config_folder": LaunchConfiguration("config_to_use"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    return LaunchDescription([
        vehicle_name_arg,
        config_to_use_arg,
        use_sim_time_arg,
        prefix_arg,
        distributed_planner,
    ])
