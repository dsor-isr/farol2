#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='vehicle',
        description='Vehicle name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    vehicle_name = LaunchConfiguration("vehicle_name")
    use_sim_time = LaunchConfiguration("use_sim_time")


    xacro_file = PathJoinSubstitution([
        FindPackageShare("farol2_description"),
        "urdf",
        [vehicle_name, TextSubstitution(text=".xacro")],
    ])

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
        # Optional xacro args if you use them inside the xacro:
        # " ",
        # "vehicle_name:=", vehicle_name,
    ])

    # robot_state_publisher (publishes /tf_static for fixed joints)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    # If you only have fixed joints, you usually DON'T need joint_state_publisher.
    # If you have non-fixed joints and no driver publishing joint states, add it.

    return LaunchDescription([
        vehicle_name_arg,
        use_sim_time_arg,
        rsp_node,
    ])