#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import (
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from hippo_control_msgs.msg import YawTarget
from nav_msgs.msg import Path
from rclpy.node import Node
from ipp_interfaces.srv import SetPath
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion


class PathFollower(Node):

    def __init__(self):
        super().__init__(node_name='path_follower')
        self.init_services()
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                 'vision_pose_cov',
                                                 self.on_pose, 1)
        self.yaw_pub = self.create_publisher(YawTarget,
                                             'yaw_controller/setpoint', 1)
        self.position_pub = self.create_publisher(
            PointStamped, 'position_controller/setpoint', 1)
        self.path_pub = self.create_publisher(Path, '~/current_path', 1)
        self.look_ahead_distance = 0.15
        self.target_index = 0
        self.path: list[PoseStamped] = None

    def init_services(self):
        self.set_path_service = self.create_service(SetPath, '~/set_path',
                                                    self.serve_set_path)
        self.path_finished_service = self.create_service(
            Trigger, '~/path_finished', self.serve_path_finished)

    def serve_set_path(self, request, response):
        self.path = request.path.poses
        self.target_index = 0
        self.get_logger().info(
            f'New path with {len(self.path)} poses has been set.')
        response.success = True
        return response

    def serve_path_finished(self, request, response: Trigger.Response):
        self.path = None
        response.success = True
        self.get_logger().info('Path finished. Going to idle mode.')
        return response

    def on_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        if not self.path:
            return
        if not self.update_setpoint(pose):
            return

        stamp = self.get_clock().now().to_msg()

        msg = YawTarget()
        msg.yaw_target = self.yaw_target
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        self.yaw_pub.publish(msg)

        msg = PointStamped(header=msg.header, point=self.position_target)
        self.position_pub.publish(msg)
        if self.path:
            msg = Path()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp
            msg.poses = self.path
            self.path_pub.publish(msg)

    def update_setpoint(self, current_pose: Pose):
        current_position = current_pose.position
        index = self.target_index
        look_ahead_square = self.look_ahead_distance**2
        if not self.path:
            self.target_index = 0
            self.position_target = None
            self.yaw_target = None
            return False
        while True:
            target_position = self.path[index].pose.position
            d = (current_position.x - target_position.x)**2 + (
                current_position.y - target_position.y)**2 + (
                    current_position.z - target_position.z)**2
            if look_ahead_square <= d:
                self.target_index = index
                break
            index += 1
            if index >= len(self.path):
                self.target_index = len(self.path) - 1
                break
        target_pose = self.path[self.target_index].pose
        self.position_target = target_pose.position
        q = target_pose.orientation
        _, _, self.yaw_target = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return True


def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
