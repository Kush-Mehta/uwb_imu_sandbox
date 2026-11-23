#!/usr/bin/env python3
"""UWB node: turns the tag's output into ROS 2 topics.

This is the simulated counterpart of the node that reads the real tag over
USB serial.  The Gazebo tag plugin publishes the same text line the real tag
prints ("[network_count,anchor_id,range_m,rx,fp,quality,delta]"); the bridge
brings it in as std_msgs/String and this node parses it.

Published
---------
/uwb/range   uwb_imu_sandbox_msgs/UwbRange   one message per ranging result,
             with the four normalised signal fields (rx, fp, quality, delta)
/uwb/ranges  std_msgs/Float64MultiArray      latest range per anchor at a fixed
             rate, NaN when an anchor has not reported for `memory_timeout`
             seconds (same layout as the real pipeline: index i = anchor_ids[i])
/uwb/truth   uwb_imu_sandbox_msgs/UwbTruth   simulation ground truth per slot
             (for scoring only)

What you can do with the data
-----------------------------
* Subscribe to /uwb/range in your own node: it has everything a range
  filter or an LOS/NLOS classifier needs (range plus signal fields).
* Subscribe to /uwb/ranges if your solver wants one fixed-size array.
* Send each result to an external process (for example an ML pipeline in its
  own Python environment) over UDP: set `udp_forward:=true` and point
  `udp_host`/`udp_port` at it.  Each datagram is one JSON object:
      {"tag": <anchor_id>, "net": .., "dist": .., "rx": .., "fp": ..,
       "q": .., "delta": .., "t": <sim time>}
  Your pipeline can send its cleaned ranges back the same way, to a node of
  your own.
* Record everything with `ros2 bag record -s mcap /uwb/range /uwb/truth /imu`
  and train offline.
"""
import json
import math
import socket

from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

from uwb_imu_sandbox.serial_format import parse_uwb_line
from uwb_imu_sandbox_msgs.msg import UwbRange, UwbTruth


class UwbNode(Node):

    def __init__(self):
        super().__init__('uwb_node')
        self.declare_parameter('serial_topic', '/sandbox/uwb/serial')
        self.declare_parameter('truth_topic', '/sandbox/uwb/truth')
        self.declare_parameter('tag_id', 1)
        self.declare_parameter('tag_frame', 'uwb_tag')
        self.declare_parameter('anchor_ids', [1, 2, 3, 4])
        self.declare_parameter('memory_timeout', 0.8)
        self.declare_parameter('array_rate', 20.0)
        self.declare_parameter('udp_forward', False)
        self.declare_parameter('udp_host', '127.0.0.1')
        self.declare_parameter('udp_port', 9999)

        gp = self.get_parameter
        self.tag_id = gp('tag_id').value
        self.tag_frame = gp('tag_frame').value
        self.anchor_ids = list(gp('anchor_ids').value)
        self.memory_timeout = float(gp('memory_timeout').value)
        self.latest = {a: (float('nan'), None) for a in self.anchor_ids}
        self.bad_lines = 0

        # best effort, depth 1: a stale range is worse than a lost one
        fast = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_range = self.create_publisher(UwbRange, '/uwb/range', 50)
        self.pub_array = self.create_publisher(Float64MultiArray, '/uwb/ranges', fast)
        self.pub_truth = self.create_publisher(UwbTruth, '/uwb/truth', 50)
        self.create_subscription(String, gp('serial_topic').value, self.on_line, 50)
        self.create_subscription(String, gp('truth_topic').value, self.on_truth, 50)
        self.create_timer(1.0 / float(gp('array_rate').value), self.publish_array)

        self.udp = None
        if gp('udp_forward').value:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (gp('udp_host').value, int(gp('udp_port').value))
            host, port = self.udp_addr
            self.get_logger().info(f'forwarding every range to udp://{host}:{port}')
        self.get_logger().info(f'tag {self.tag_id}: anchors {self.anchor_ids}')

    # ------------------------------------------------------------------
    def on_line(self, msg: String):
        r = parse_uwb_line(msg.data)
        if r is None:
            self.bad_lines += 1
            if self.bad_lines in (1, 10, 100) or self.bad_lines % 1000 == 0:
                self.get_logger().warn(
                    f'ignored {self.bad_lines} malformed line(s), last: {msg.data!r}')
            return
        now = self.get_clock().now()
        out = UwbRange()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.tag_frame
        out.tag_id = int(self.tag_id)
        out.anchor_id = r.anchor_id
        out.network_count = min(r.network_count, 255)
        out.range = r.range_m
        out.rx_level = r.rx
        out.fp_level = r.fp
        out.quality = r.quality
        out.delta_level = r.delta
        self.pub_range.publish(out)
        if r.anchor_id in self.latest:
            self.latest[r.anchor_id] = (r.range_m, now)

        # ---- optional: hand the raw result to an external pipeline --------
        if self.udp is not None:
            payload = {'tag': r.anchor_id, 'net': r.network_count, 'dist': r.range_m,
                       'rx': r.rx, 'fp': r.fp, 'q': r.quality, 'delta': r.delta,
                       't': now.nanoseconds * 1e-9}
            try:
                self.udp.sendto(json.dumps(payload).encode('utf-8'), self.udp_addr)
            except OSError as e:
                self.get_logger().warn(f'udp send failed: {e}', throttle_duration_sec=5.0)

    def publish_array(self):
        now = self.get_clock().now()
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='anchor', size=len(self.anchor_ids),
                                                  stride=len(self.anchor_ids)))
        data = []
        for a in self.anchor_ids:
            dist, stamp = self.latest[a]
            fresh = stamp is not None and (now - stamp).nanoseconds * 1e-9 <= self.memory_timeout
            data.append(dist if fresh else float('nan'))
        msg.data = data
        self.pub_array.publish(msg)

    def on_truth(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        t = UwbTruth()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.tag_id = int(d.get('tag', self.tag_id))
        t.anchor_id = int(d['anchor'])
        t.true_range = float(d['true_range'])
        t.line_of_sight = bool(d['los'])
        t.nlos_weight = float(d['nlos_weight'])
        t.obstacles = int(d['obstacles'])
        t.clearance = float(d['clearance'])
        nu = float(d['nu'])
        t.fresnel_nu = nu if math.isfinite(nu) else -10.0
        t.outcome = str(d['outcome'])
        t.tag_position = Point(x=d['tag_pos'][0], y=d['tag_pos'][1], z=d['tag_pos'][2])
        t.anchor_position = Point(x=d['anchor_pos'][0], y=d['anchor_pos'][1], z=d['anchor_pos'][2])
        self.pub_truth.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = UwbNode()
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
