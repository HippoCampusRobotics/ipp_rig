#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hippo_control_msgs.msg import ActuatorSetpoint, YawTarget
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion


class YawController(Node):

    def __init__(self):
        super().__init__(node_name='yaw_controller')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        # default value for the yaw setpoint
        self.setpoint = math.pi / 2.0
        self.setpoint_timed_out = True

        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)
        self.setpoint_sub = self.create_subscription(YawTarget,
                                                     topic='~/setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=qos)
        self.timeout_timer = self.create_timer(0.5, self.on_setpoint_timeout)

        self.torque_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='torque_setpoint',
                                                qos_profile=1)

        self.declare_parameter('kp', 0.1)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.last_time = None
        self.integral = 0.0
        self.last_error = 0.0

    def reset_pid(self):
        self.last_time = None
        self.integral = 0.0
        self.last_error = 0.0

    def on_setpoint_timeout(self):
        self.timeout_timer.cancel()
        self.get_logger().warn('Setpoint timed out. Waiting for new setpoints')
        self.setpoint_timed_out = True
        self.reset_pid()

    def wrap_pi(self, value: float):
        """Normalize the angle to the range [-pi; pi]."""
        if (-math.pi < value) and (value < math.pi):
            return value
        range = 2 * math.pi
        num_wraps = math.floor((value + math.pi) / range)
        return value - range * num_wraps

    def on_setpoint(self, msg: YawTarget):
        self.timeout_timer.reset()
        if self.setpoint_timed_out:
            self.get_logger().info('Setpoint received! Getting back to work.')
            self.reset_pid()
        self.setpoint_timed_out = False
        self.setpoint = self.wrap_pi(msg.yaw_target)

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        if self.setpoint_timed_out:
            return
        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert the quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        #yaw = self.wrap_pi(yaw)

        control_output = self.compute_control_output(yaw)
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.publish_control_output(control_output, timestamp)

    def compute_control_output(self, yaw):
        # very important: normalize the angle error!
        error = self.wrap_pi(self.setpoint - yaw)

        now = self.get_clock().now()
        if self.last_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9

        if dt > 0.0:
            self.integral += error * dt
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0

        self.last_time = now
        self.last_error = error

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        return kp * error + ki * self.integral + kd * derivative

    def publish_control_output(self, control_output: float,
                               timestamp: rclpy.time.Time):
        msg = ActuatorSetpoint()
        msg.header.stamp = timestamp.to_msg()
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False  # yaw is the rotation around the vehicle's z axis

        msg.z = control_output
        self.torque_pub.publish(msg)


def main():
    rclpy.init()
    node = YawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
