#!/usr/bin/env python3
"""Demo driver: drives the robot around the warehouse on its own.

It follows a fixed loop of waypoints through the main aisle, the dock area
and past the rack ends, so every anchor goes through LOS, near-edge and NLOS
conditions.  It steers with the simulator's exact pose (/ground_truth/odom),
which is fine for a demo; it is not part of any localization.
For another world, pass your own loop with the `waypoints` parameter.
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# x, y in the world frame [m]
DEFAULT_LOOP = [
    2.6, 0.0,
    4.4, -1.0,
    4.0, -1.7,
    1.5, -1.7,
    -4.4, -1.6,
    -4.9, 0.0,
    -4.4, 1.5,
    1.5, 0.9,
    0.0, 0.0,
]


class DemoDriver(Node):

    def __init__(self):
        super().__init__('demo_driver')
        self.declare_parameter('waypoints', DEFAULT_LOOP)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('loop', True)
        wp = list(self.get_parameter('waypoints').value)
        if len(wp) < 2 or len(wp) % 2:
            raise ValueError('waypoints must be a flat list x1, y1, x2, y2, ...')
        self.wps = [(wp[i], wp[i + 1]) for i in range(0, len(wp), 2)]
        self.speed = float(self.get_parameter('speed').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.i = 0
        self.pose = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info(f'driving {len(self.wps)} waypoints at {self.speed} m/s')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)

    def step(self):
        cmd = Twist()
        if self.pose is None or self.i >= len(self.wps):
            self.pub.publish(cmd)
            return
        x, y, yaw = self.pose
        gx, gy = self.wps[self.i]
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.25:
            self.i += 1
            if self.i >= len(self.wps) and self.loop:
                self.i = 0
            return
        err = math.atan2(gy - y, gx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        cmd.angular.z = max(-1.2, min(1.2, 2.0 * err))
        cmd.linear.x = self.speed * max(0.0, math.cos(err)) ** 2 * min(1.0, dist / 0.6 + 0.3)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DemoDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
