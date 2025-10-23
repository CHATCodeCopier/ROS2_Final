from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("smart_car")

    # --- copied approach from your other launch file ---
    xacro_path = os.path.join(pkg_share, "urdf", "smartcar.urdf.xacro")
    robot_description = Command(["xacro", " ", xacro_path])

    # --- Gazebo world path (installed via CMake into share/smart_car/world) ---
    world_path = os.path.join(pkg_share, "world", "YOUR_WORLD.world")  # <-- put your .world filename

    # Gazebo server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items()
    )

    # Robot description publisher (same pattern as your existing launch)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
        output="screen",
    )

    # Optional: joint GUI (copied from your launch)
    joint_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Spawn the robot into Gazebo from the robot_description topic
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description",
                   "-entity", "smartcar"],
        output="screen",
    )

    return LaunchDescription([gazebo, robot_state_pub, joint_gui, spawn])

