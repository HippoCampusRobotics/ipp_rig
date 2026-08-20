#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hippo_control_msgs.msg import ActuatorSetpoint
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion


class RollController(Node):

    def __init__(self):
        super().__init__(node_name='roll_controller')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        # roll is always held level, there is no setpoint topic
        self.setpoint = 0.0

        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)

        self.torque_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='torque_setpoint',
                                                qos_profile=1)

        self.declare_parameter('kp', 0.1)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.last_time = None
        self.integral = 0.0
        self.last_error = 0.0

    def wrap_pi(self, value: float):
        """Normalize the angle to the range [-pi; pi]."""
        if (-math.pi < value) and (value < math.pi):
            return value
        range = 2 * math.pi
        num_wraps = math.floor((value + math.pi) / range)
        return value - range * num_wraps

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        control_output = self.compute_control_output(roll)
        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.publish_control_output(control_output, timestamp)

    def compute_control_output(self, roll):
        error = self.wrap_pi(self.setpoint - roll)

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
        msg.ignore_x = False  # roll is the rotation around the vehicle's x axis
        msg.ignore_y = True
        msg.ignore_z = True

        msg.x = control_output
        self.torque_pub.publish(msg)


def main():
    rclpy.init()
    node = RollController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
