from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("smart_car")

    # Robot description from Xacro (robust: resolves xacro and inserts a space)
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "smartcar.urdf.xacro"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str
    )

    # Debug log so you can see which xacro file is used
    log_xacro = LogInfo(msg=["Using xacro file: ", xacro_file])

    # World file
    world_path = os.path.join(pkg_share, "world", "smalltown.world")

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Spawn the robot in Gazebo
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "smartcar", "-topic", "robot_description"],
        output="screen",
    )

    # Wheel odometry node (scripts/ installed as executable)
    odom_node = Node(
        package="smart_car",
        executable="wheel_odom.py",
        name="wheel_odom",
        output="screen",
        parameters=[{
            "wheel_radius": 0.032,
            "wheelbase": 0.257,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "use_sim_time": True,
        }],
        # Keep ONLY if your publisher uses /smartcar/vehicle_status (no underscore).
        remappings=[
            ("/smart_car/vehicle_status", "/smartcar/vehicle_status"),
        ],
    )

 

    return LaunchDescription([
        log_xacro,
        gazebo,
        robot_state_pub,
        spawn,
        odom_node,
        basefoot_tf,   # <— add this
    ])

