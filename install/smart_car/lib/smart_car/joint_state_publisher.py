#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smartcar_msgs.msg import Status   # battery_*, steering_angle_rad, engine_speed_rpm

class SmartCarJointStatePublisher(Node):
    """
    Publish /joint_states from SmartCAR Status:
      - steering_angle_rad (rad)
      - engine_speed_rpm   (rpm)
    """
    def __init__(self) -> None:
        super().__init__('smartcar_joint_state_publisher')

        # Do NOT declare use_sim_time here; launch sets it.

        # Parameters you own:
        self.declare_parameter('wheel_radius', 0.032)            # m
        self.declare_parameter('status_topic', '/smartcar/vehicle_status')

        self.r = float(self.get_parameter('wheel_radius').value)
        status_topic = str(self.get_parameter('status_topic').value)

        # URDF joint names (must match your xacro)
        self.joint_names = [
            'front_left_wheel_steer_joint',
            'front_right_wheel_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
        ]

        # state
        self.last_time: Optional[rclpy.time.Time] = None
        self.steer = 0.0
        self.rpm = 0.0
        self.theta = {
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'back_left_wheel_joint': 0.0,
            'back_right_wheel_joint': 0.0,
        }

        # pubs/subs
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Status, status_topic, self.on_status, 10)
        self.timer = self.create_timer(1/50.0, self.on_timer)  # 50 Hz

        self.get_logger().info(f'Listening: {status_topic}')

    def on_status(self, msg: Status) -> None:
        self.steer = float(msg.steering_angle_rad)
        self.rpm   = float(msg.engine_speed_rpm)

    def on_timer(self) -> None:
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Engine RPM -> wheel angular speed [rad/s] (assume 1:1 for viz)
        omega = (self.rpm * 2.0 * math.pi) / 60.0

        # integrate wheel spin
        for k in self.theta:
            self.theta[k] = (self.theta[k] + omega * dt) % (2.0 * math.pi)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = [
            self.steer,                          # front_left_wheel_steer_joint
            self.steer,                          # front_right_wheel_steer_joint
            self.theta['front_left_wheel_joint'],
            self.theta['front_right_wheel_joint'],
            self.theta['back_left_wheel_joint'],
            self.theta['back_right_wheel_joint'],
        ]
        js.velocity = [0.0, 0.0, omega, omega, omega, omega]
        self.pub_js.publish(js)

def main() -> None:
    rclpy.init()
    node = SmartCarJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

