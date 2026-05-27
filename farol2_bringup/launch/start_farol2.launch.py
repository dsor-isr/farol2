# Explicit, hand-edited Farol stack orchestrator (replaces old process.yaml)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments for vehicle identity and config
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name', default_value='magicelectric', description='Vehicle name.'
    )
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id', default_value='0', description='Vehicle ID.'
    )
    
    config_to_use_arg = DeclareLaunchArgument(
        'config_to_use', default_value='default', description='Config folder to use.'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time for Farol stack.'
    )

    
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
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
            'sample_and_hold': 'false',
            'filter_node': 'true',
        }.items()
    )

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_description'), 'launch', 'tf_static.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_name': LaunchConfiguration('vehicle_name'),
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
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
            'static_thruster_allocation': 'false',
            'thruster_rudder_allocation': 'true',
        }.items()
    )

    inner_loop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_inner_loop'), 'launch', 'farol2_inner_loop.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
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
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
        }.items()
    )

    path_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_path_following'), 'launch', 'farol2_path_following.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
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
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
        }.items()
    )

    console = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('farol2_console'), 'launch', 'farol2_console.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'vehicle_id': LaunchConfiguration('vehicle_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_to_use': LaunchConfiguration('config_to_use'),
        }.items()
    )

    # --- END EDIT ---

    return LaunchDescription([
        # args
        vehicle_name_arg,
        vehicle_id_arg,
        config_to_use_arg,
        use_sim_time_arg,
        # launches (one per package)
        nav,
        tf_static,
        allocation,
        inner_loop,
        waypoint,
        path_following,
        planning,
        console,
    ])
