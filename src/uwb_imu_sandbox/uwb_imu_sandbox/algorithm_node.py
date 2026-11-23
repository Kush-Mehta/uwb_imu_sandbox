#!/usr/bin/env python3
"""Algorithm node: an empty slot for YOUR localization algorithm.

Nothing is implemented here on purpose.  The node already receives every
input the sandbox provides; fill in the callbacks with your own range
filter, LOS/NLOS classifier, position solver and fusion filter.

Inputs
------
/uwb/range    uwb_imu_sandbox_msgs/UwbRange  one ranging result at a time
              (range [m] + normalised rx/fp/quality/delta fields)
/uwb/ranges   std_msgs/Float64MultiArray     latest range per anchor, NaN if stale
/imu          sensor_msgs/Imu                ~100 Hz, real-sensor-like errors

Anchor positions in the test room (world frame, metres):
    1: ( 3.425,  5.136, 1.02)    2: (-3.998,  5.136, 1.02)
    3: (-3.998, -5.136, 1.02)    4: ( 3.425, -5.136, 1.02)
The tag antenna is 0.50 m above the floor.

Suggested output
----------------
Publish your estimate as geometry_msgs/PoseWithCovarianceStamped or
nav_msgs/Odometry (frame 'world').  Ground truth for scoring is on
/ground_truth/odom (nav_msgs/Odometry) and /uwb/truth.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

from uwb_imu_sandbox_msgs.msg import UwbRange


class AlgorithmNode(Node):

    def __init__(self):
        super().__init__('algorithm_node')
        self.create_subscription(UwbRange, '/uwb/range', self.on_range, 50)
        # /uwb/ranges is published best-effort (a stale array is worse than a lost one)
        self.create_subscription(Float64MultiArray, '/uwb/ranges', self.on_ranges,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, 100)
        self.get_logger().info('algorithm_node is running with no algorithm (template)')

    def on_range(self, msg: UwbRange):
        # TODO: your range filter / LOS-NLOS classifier goes here
        pass

    def on_ranges(self, msg: Float64MultiArray):
        # TODO: your position solver goes here
        pass

    def on_imu(self, msg: Imu):
        # TODO: your prediction step (dead reckoning, EKF predict, ...) goes here
        pass


def main(args=None):
    rclpy.init(args=args)
    node = AlgorithmNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
