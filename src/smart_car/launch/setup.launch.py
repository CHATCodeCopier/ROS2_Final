from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('smart_car')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    xacro_file  = PathJoinSubstitution([pkg_share, 'urdf', 'smartcar.urdf.xacro'])
    world_file  = PathJoinSubstitution([pkg_share, 'world', 'smalltown.world'])
    params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    map_yaml    = PathJoinSubstitution([pkg_share, 'nav2_map', 'smalltown_world.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'Rviz', 'smartcar.rviz'])

    # Use Nav2 BT without clear-costmap recoveries
    bt_tree = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning.xml'
    ])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Comment these if your VM has good 3D acceleration
    env_sw_render = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'none'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world_file}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_smartcar',
        output='screen',
        arguments=['-entity', 'smartcar', '-topic', 'robot_description'],
    )

    wheel_odom = Node(
        package='smart_car',
        executable='wheel_odom.py',
        name='wheel_odom',
        output='screen',
        parameters=[{
            'wheel_radius': 0.032,
            'wheelbase': 0.257,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': True,
        }],
        remappings=[('/smart_car/vehicle_status', '/smartcar/vehicle_status')],
    )

    jsp = Node(
        package='smart_car',
        executable='joint_state_publisher.py',
        name='smartcar_joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'status_topic': '/smartcar/vehicle_status',
            'wheel_radius': 0.032,
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'default_bt_xml_filename': bt_tree},
                    params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'bt_navigator',
                'waypoint_follower',
                'behavior_server',
            ],
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf_localization.yaml'])],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    logs = [
        LogInfo(msg=['[setup] params: ', params_file]),
        LogInfo(msg=['[setup] map: ',    map_yaml]),
        LogInfo(msg=['[setup] rviz: ',   rviz_config]),
        LogInfo(msg=['[setup] xacro: ',  xacro_file]),
    ]

    return LaunchDescription(
        [declare_use_sim_time] +
        env_sw_render +
        logs + [
            gazebo,
            TimerAction(period=1.0, actions=[rsp]),
            TimerAction(period=2.0, actions=[spawn]),
            TimerAction(period=2.5, actions=[wheel_odom, jsp]),
            TimerAction(period=3.0, actions=[ekf]),
            TimerAction(period=3.5, actions=[
                map_server, amcl, controller_server, planner_server,
                bt_navigator, waypoint_follower, behavior_server, lifecycle_manager
            ]),
            TimerAction(period=5.0, actions=[rviz]),
        ]
    )

