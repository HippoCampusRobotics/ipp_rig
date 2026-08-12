#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
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

        # Sensor mount offset relative to base_link, same values as
        # hippo_common's ir_sensor tf. Path waypoints are planned for the
        # sensor position, so this has to be subtracted from the setpoint
        # sent to position_controller.
        self.declare_parameter('ir_sensor.x', 0.0)
        self.declare_parameter('ir_sensor.y', 0.0)
        self.declare_parameter('ir_sensor.z', 0.0)
        self.ir_sensor_offset_x = self.get_parameter('ir_sensor.x').value
        self.ir_sensor_offset_y = self.get_parameter('ir_sensor.y').value
        self.ir_sensor_offset_z = self.get_parameter('ir_sensor.z').value

        self.look_ahead_distance = 0.4
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
        reached_end = False
        while True:
            target_position = self.path[index].pose.position
            d = (current_position.x - target_position.x)**2 + (
                current_position.y - target_position.y)**2
            if look_ahead_square <= d:
                self.target_index = index
                break
            index += 1
            if index >= len(self.path):
                self.target_index = len(self.path) - 1
                reached_end = True
                break
        target_pose = self.path[self.target_index].pose
        self.position_target = self._apply_sensor_offset(
            target_pose.position, current_pose.orientation)
        if reached_end:
            q = target_pose.orientation
            _, _, self.yaw_target = euler_from_quaternion([q.x, q.y, q.z, q.w])
        else:
            self.yaw_target = math.atan2(
                target_pose.position.y - current_position.y,
                target_pose.position.x - current_position.x)
        return True

    def _apply_sensor_offset(self, target: Point,
                              orientation: Quaternion) -> Point:
        # target is the desired sensor position. position_controller drives
        # base_link though, so the yaw-rotated sensor offset is subtracted
        # here to get the base_link setpoint. Roll/pitch are ignored, the
        # BlueROV stabilizes itself in those axes.
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        dx = (math.cos(yaw) * self.ir_sensor_offset_x -
              math.sin(yaw) * self.ir_sensor_offset_y)
        dy = (math.sin(yaw) * self.ir_sensor_offset_x +
              math.cos(yaw) * self.ir_sensor_offset_y)
        return Point(x=target.x - dx,
                     y=target.y - dy,
                     z=target.z - self.ir_sensor_offset_z)


def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
