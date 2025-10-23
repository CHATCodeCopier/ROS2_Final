from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("smart_car")
    xacro_path = os.path.join(pkg_share, "urdf", "smartcar.urdf.xacro")

    # same as before: xacro -> robot_description
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "use_sim_time": True,             # keep sim time here
                "robot_description": robot_description
            }],
            output="screen",
        ),
        # REPLACE the GUI with your custom publisher
        Node(
            package="smart_car",
            executable="joint_state_publisher.py",
            name="smartcar_joint_state_publisher",
            parameters=[{
                "use_sim_time":True,
                "status_topic": "/smartcar/vehicle_status",
                "wheel_radius": 0.032
            }],
            output="screen",
        ),
    ])

