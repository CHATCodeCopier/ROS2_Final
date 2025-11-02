#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from smartcar_msgs.msg import Status
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WheelOdom(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odom')

        # ---------------- Params (can override in launch) ----------------
        self.declare_parameter('wheel_radius', 0.032)   # meters
        self.declare_parameter('wheelbase', 0.257)      # meters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)     # disable to avoid fighting Gazebo
        self.declare_parameter('status_topic', '/smart_car/vehicle_status')

        self.R = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        status_topic = str(self.get_parameter('status_topic').value)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_t = None

        # ---------------- Pub/Sub ----------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.status_sub = self.create_subscription(
            Status,
            status_topic,
            self.on_status,
            sensor_qos
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/smart_car/wheel/odom',
            10
        )

        # Keep a TF broadcaster available, but we may not use it
        self.tfb = TransformBroadcaster(self)

        self.get_logger().info(
            f'wheel_odom running | R={self.R:.4f} m, L={self.L:.4f} m, '
            f'frames: {self.odom_frame}->{self.base_frame} | publish_tf={self.publish_tf} | '
            f'status_topic={status_topic}'
        )

    def on_status(self, msg: Status) -> None:
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        t_sec = now_time.nanoseconds * 1e-9

        if self._last_t is None:
            self._last_t = t_sec
            return
        dt = max(1e-6, t_sec - self._last_t)
        self._last_t = t_sec

        # RPM -> linear speed (circumference = 2πR)
        rpm = float(msg.engine_speed_rpm)
        v = rpm * 2.0 * math.pi * self.R / 60.0

        # Ackermann small-angle model
        delta = float(msg.steering_angle_rad)
        yaw_rate = v * math.tan(delta) / self.L

        # Integrate pose
        self.yaw += yaw_rate * dt
        # wrap to [-pi, pi)
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # --------- TF (optional): odom -> base_* ----------
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now_msg
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = yaw_to_quat(self.yaw)
            self.tfb.sendTransform(tf)

        # --------- Odometry message ----------
        od = Odometry()
        od.header.stamp = now_msg
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame

        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation = yaw_to_quat(self.yaw)

        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = yaw_rate

        # Covariances must be floats and length 36
        pose_cov = [0.0] * 36
        pose_cov[0]  = 1.0
        pose_cov[7]  = 1.0
        pose_cov[14] = 1e5
        pose_cov[21] = 1e5
        pose_cov[28] = 1e5
        pose_cov[35] = 0.5
        od.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.1
        twist_cov[7]  = 1e5
        twist_cov[14] = 1e5
        twist_cov[21] = 1e5
        twist_cov[28] = 1e5
        twist_cov[35] = 0.1
        od.twist.covariance = twist_cov

        self.odom_pub.publish(od)


def main() -> None:
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

