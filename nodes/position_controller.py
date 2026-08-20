#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from hippo_control_msgs.msg import ActuatorSetpoint
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class PositionController(Node):

    def __init__(self):
        super().__init__(node_name='position_controller')

        self.thrust_pub = self.create_publisher(ActuatorSetpoint,
                                                'thrust_setpoint', 1)
        self.position_setpoint_sub = self.create_subscription(
            PointStamped, '~/setpoint', self.on_position_setpoint, 1)
        self.setpoint = Point()
        self.setpoint_timed_out = True
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                 'vision_pose_cov',
                                                 self.on_pose, 1)
        self.timeout_timer = self.create_timer(0.5, self.on_setpoint_timeout)
        self.pose_counter = 0

        self._declare_parameters()

        self.last_time = None
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0

    def _declare_parameters(self):
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('kd_xy', 0.2)

        self.declare_parameter('kp_z', 2.0)
        self.declare_parameter('ki_z', 0.1)
        self.declare_parameter('kd_z', 2.0)

        self.declare_parameter('max_thrust_xy', 0.3)

    def on_setpoint_timeout(self):
        self.timeout_timer.cancel()
        self.get_logger().warn('setpoint timed out. waiting for new setpoints.')
        self.setpoint_timed_out = True
        self.reset_pid()

    def on_position_setpoint(self, msg: PointStamped):
        self.timeout_timer.reset()
        if self.setpoint_timed_out:
            self.get_logger().info('Setpoint received! Getting back to work.')
            self.reset_pid()
        self.setpoint_timed_out = False
        self.setpoint = msg.point

    def reset_pid(self):
        self.last_time = None
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0

    def on_pose(self, msg: PoseWithCovarianceStamped):
        if self.setpoint_timed_out:
            return
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.apply_control(position, yaw)

    def apply_control(self, position: Point, yaw: float):
        now = self.get_clock().now()

        # Errors
        x_error = self.setpoint.x - position.x
        y_error = self.setpoint.y - position.y
        z_error = self.setpoint.z - position.z

        # Integral and derivatives
        if self.last_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9

        if dt > 0.0:
            self.integral_z += z_error * dt
            derivative_x = (x_error - self.last_error_x) / dt
            derivative_y = (y_error - self.last_error_y) / dt
            derivative_z = (z_error - self.last_error_z) / dt
        else:
            derivative_x = 0.0
            derivative_y = 0.0
            derivative_z = 0.0

        # xy PD --------------
        kp_xy = self.get_parameter('kp_xy').value
        kd_xy = self.get_parameter('kd_xy').value
        x = kp_xy * x_error + kd_xy * derivative_x
        y = kp_xy * y_error + kd_xy * derivative_y

        # z PID --------------
        kp_z = self.get_parameter('kp_z').value
        ki_z = self.get_parameter('ki_z').value
        kd_z = self.get_parameter('kd_z').value
        z = kp_z * z_error + ki_z * self.integral_z + kd_z * derivative_z

        # Update Errors
        self.last_time = now
        self.last_error_x = x_error
        self.last_error_y = y_error
        self.last_error_z = z_error

        # Thrust normalisation in xy plane
        thrust_x = math.cos(-yaw) * x - math.sin(-yaw) * y
        thrust_y = math.sin(-yaw) * x + math.cos(-yaw) * y
        max_thrust_xy = self.get_parameter('max_thrust_xy').value
        thrust_xy_norm = math.hypot(thrust_x, thrust_y)
        if thrust_xy_norm > max_thrust_xy:
            scale = max_thrust_xy / thrust_xy_norm
            thrust_x *= scale
            thrust_y *= scale

        msg = ActuatorSetpoint()
        msg.header.stamp = now.to_msg()
        msg.x = thrust_x
        msg.y = thrust_y
        msg.z = z

        self.thrust_pub.publish(msg)


def main():
    rclpy.init()
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
