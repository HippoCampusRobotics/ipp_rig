#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class VisionPoseBridge(Node):

    def __init__(self):
        super().__init__(node_name='vision_pose_bridge')

        sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             depth=1)
        self.odometry_sub = self.create_subscription(
            msg_type=Odometry,
            topic='ground_truth/odometry',
            callback=self.on_odometry,
            qos_profile=sub_qos)
        self.vision_pose_pub = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov_bridged',
            qos_profile=1)

    def on_odometry(self, msg: Odometry):
        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.vision_pose_pub.publish(pose)


def main():
    rclpy.init()
    node = VisionPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
