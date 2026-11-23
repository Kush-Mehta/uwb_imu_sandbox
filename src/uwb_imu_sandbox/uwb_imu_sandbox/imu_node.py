#!/usr/bin/env python3
"""IMU node: publishes the simulated IMU on /imu.

This is the simulated counterpart of the node that reads the real IMU over
I2C.  The RealisticImu Gazebo plugin already does all the sensor modelling
(internal sample clocks, turn-on bias, vibration jitter, output resolution);
this node only takes the bridged message and publishes it on /imu with the
frame your stack expects, exactly like the hardware node does.

Published
---------
/imu  sensor_msgs/Imu  ~100 Hz, frame `frame_id` (default base_link).
      Like the real sensor, many consecutive messages repeat the last
      internal sample: the accelerometer refreshes about every 30 ms and the
      gyroscope and orientation about every 50 ms.
      Covariances are left at zero (unknown), as on the hardware node.

Ground truth (no errors) is on /sandbox/imu/truth for scoring.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('input_topic', '/sandbox/imu/raw')
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(Imu, '/imu', 50)
        self.create_subscription(Imu, self.get_parameter('input_topic').value, self.on_imu, 50)

    def on_imu(self, msg: Imu):
        # keep the sensor's own timestamp (simulation time of the reading)
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
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
