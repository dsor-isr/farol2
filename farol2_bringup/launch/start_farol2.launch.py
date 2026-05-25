# Explicit, hand-edited Farol stack orchestrator (replaces old process.yaml)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments for vehicle identity and config
    name_arg = DeclareLaunchArgument(
        'name', default_value='magicelectric', description='Vehicle name.'
    )
    id_arg = DeclareLaunchArgument(
        'id', default_value='0', description='Vehicle ID.'
    )
    config_package_arg = DeclareLaunchArgument(
        'config_package', default_value='personal_bringup', description='Config package.'
    )
    config_package_path_real_arg = DeclareLaunchArgument(
        'config_package_path_real',
        default_value=PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('config_package')),
            '..', '..', '..', '..',
            'src',
            LaunchConfiguration('config_package')
        ]),
        description='Path to the config package in the src folder.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time for Farol stack.'
    )

    vehicle_ns = PythonExpression([
        "'", LaunchConfiguration('name'), "' + '", LaunchConfiguration('id'), "'"
    ])

    # --- EDIT BELOW: This is your new stack definition ---
    # Add/remove IncludeLaunchDescription or Node actions as needed
    # Example: Navigation

    # Example: add custom arguments per subsystem as needed
    # Just add more key-value pairs to the launch_arguments dict

    ########################################################################################
    ## NEW PROCESS LAUNCHES GO HERE (copy-paste from old process.yaml and edit as needed) ##
    ########################################################################################

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_nav'), 'launch', 'farol2_nav.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
            'sample_and_hold': 'true',
            'low_pass': 'false',
        }.items()
    )

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_description'), 'launch', 'tf_static.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    allocation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_allocation'), 'launch', 'farol2_allocation.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
            'thruster_allocation': 'true',
        }.items()
    )

    inner_loop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_inner_loop'), 'launch', 'farol2_inner_loop.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
            'pid': 'true',
            'open_loop': 'false',
        }.items()
    )

    waypoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_waypoint'), 'launch', 'farol2_waypoint.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
        }.items()
    )

    path_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_path_following'), 'launch', 'farol2_path_following.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
            # Example: add custom args here if needed
        }.items()
    )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_planning'), 'launch', 'farol2_planning.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
        }.items()
    )

    console = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_console'), 'launch', 'farol2_console.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_ns': vehicle_ns,
            'vehicle_name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_package_path_share': FindPackageShare(LaunchConfiguration('config_package')),
            'config_package_path_real': LaunchConfiguration('config_package_path_real'),
        }.items()
    )

    # --- END EDIT ---

    return LaunchDescription([
        name_arg,
        id_arg,
        config_package_arg,
        config_package_path_real_arg,
        use_sim_time_arg,
        nav,
        tf_static,
        allocation,
        inner_loop,
        waypoint,
        path_following,
        planning,
        console,
    ])
