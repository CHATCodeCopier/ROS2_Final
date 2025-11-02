from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui_arg      = LaunchConfiguration('gui')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_gui         = DeclareLaunchArgument('gui', default_value='true')

    # Paths
    pkg = FindPackageShare('smart_car')
    gz  = FindPackageShare('gazebo_ros')

    xacro_file  = PathJoinSubstitution([pkg, 'urdf', 'smartcar.urdf.xacro'])
    world_file  = PathJoinSubstitution([pkg, 'world', 'smalltown.world'])
    params_file = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])
    map_yaml    = PathJoinSubstitution([pkg, 'nav2_map', 'smalltown_world.yaml'])
    rviz_cfg    = PathJoinSubstitution([pkg, 'Rviz', 'smartcar.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str
    )

    # Softrender env for VMs
    env = GroupAction([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'none'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
    ])

    # 1) Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([gz, 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': world_file, 'gui': gui_arg}.items()
    )

    # 2) RSP
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    # 3) Spawn
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_smartcar', output='screen',
        arguments=['-entity', 'smartcar', '-topic', 'robot_description'],
    )

    # 4) Your nodes
    wheel_odom = Node(
        package='smart_car', executable='wheel_odom.py', name='wheel_odom', output='screen',
        parameters=[{'wheel_radius': 0.032, 'wheelbase': 0.257, 'odom_frame': 'odom',
                     'base_frame': 'base_link',          # ← was base_footprint
                     'publish_tf': True,                # ← new: stop TF from wheel_odom
                     'use_sim_time': use_sim_time}],     # ← follow launch arg
        remappings=[('/smart_car/vehicle_status', '/smartcar/vehicle_status')],
    )
    jsp = Node(
        package='smart_car', executable='joint_state_publisher.py',
        name='smartcar_joint_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'status_topic': '/smartcar/vehicle_status', 'wheel_radius': 0.032}],
    )

    # 5) EKF
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'ekf_localization.yaml'])],
    )

    # 6) Map + AMCL
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params_file],
    )

    # 7) Nav2 servers + lifecycle
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=[{'use_sim_time': use_sim_time}, params_file],
    )
    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[{'use_sim_time': use_sim_time}, params_file],
    )
    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[{'use_sim_time': use_sim_time}, params_file],
    )
    waypoint = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen', parameters=[{'use_sim_time': use_sim_time}, params_file],
    )
    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[{'use_sim_time': use_sim_time}, params_file],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server','amcl','controller_server','planner_server',
                                    'bt_navigator','waypoint_follower','behavior_server']}],
    )

    # 8) RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],   # ← was hardcoded True
    )

    # Strict timeline (adjust seconds for your VM)
    chain = [
        TimerAction(period=0.0, actions=[gazebo]),                 # t=0   Gazebo
        TimerAction(period=2.0, actions=[rsp]),                    # t=2   RSP
        TimerAction(period=3.5, actions=[spawn]),                  # t=3.5 Spawn
        TimerAction(period=4.5, actions=[wheel_odom, jsp]),        # t=4.5 Odom + Joints
        TimerAction(period=5.5, actions=[ekf]),                    # t=5.5 EKF
        TimerAction(period=6.5, actions=[map_server, amcl]),       # t=6.5 Map + AMCL
        TimerAction(period=8.0, actions=[planner, controller, bt_nav, waypoint, behaviors, lifecycle]),  # t=8 Nav2
        TimerAction(period=10.0, actions=[rviz]),                  # t=10  RViz (last)
    ]

    return LaunchDescription([declare_use_sim_time, declare_gui, env, *chain])
