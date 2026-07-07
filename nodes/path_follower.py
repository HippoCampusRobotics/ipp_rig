#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import (
    Point,
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
        self.look_ahead_distance = 0.3
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
        if not self.path:
            self.target_index = 0
            self.position_target = None
            self.yaw_target = None
            return False

        current_position = current_pose.position
        r = self.look_ahead_distance
        n = len(self.path)

        lookahead_point = None
        for i in range(self.target_index, n - 1):
            p0 = self.path[i].pose.position
            p1 = self.path[i + 1].pose.position

            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            fx = p0.x - current_position.x
            fy = p0.y - current_position.y
            fz = p0.z - current_position.z

            a = dx * dx + dy * dy + dz * dz
            if a < 1e-9:
                continue
            b = 2.0 * (fx * dx + fy * dy + fz * dz)
            c = fx * fx + fy * fy + fz * fz - r * r

            discriminant = b * b - 4.0 * a * c
            if discriminant < 0.0:
                continue

            sqrt_disc = math.sqrt(discriminant)
            t2 = (-b + sqrt_disc) / (2.0 * a)
            t1 = (-b - sqrt_disc) / (2.0 * a)

            if 0.0 <= t2 <= 1.0:
                t = t2
            elif 0.0 <= t1 <= 1.0:
                t = t1
            else:
                continue

            self.target_index = i
            lookahead_point = Point(x=p0.x + t * dx,
                                    y=p0.y + t * dy,
                                    z=p0.z + t * dz)
            break

        if lookahead_point is None:
            target_pose = self.path[-1].pose
            self.position_target = target_pose.position
            q = target_pose.orientation
            _, _, self.yaw_target = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return True

        self.position_target = lookahead_point
        self.yaw_target = math.atan2(lookahead_point.y - current_position.y,
                                     lookahead_point.x - current_position.x)
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
