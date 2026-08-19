#!/usr/bin/env python3
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from scalar_field_interfaces.msg import ScalarMeasurement
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

NUM_CHANNELS = 9


def fuse_ir_channels(channel_values: list) -> float:
    return sum(channel_values) / len(channel_values)


class IrMeasurementBridge(Node):
    """Turns 9 continuously streaming real IR channels into the same
    ir_measurement / trigger_ir_measurement interface the sim adapter
    (ir_measurement_sim_node.py) already provides. Nothing downstream
    needs to know whether it is talking to sim or real hardware.
    """

    def __init__(self):
        super().__init__(node_name='ir_measurement_bridge')

        default_topics = [
            f'/photodiode_array/diode_{i + 1}' for i in range(NUM_CHANNELS)]
        self.declare_parameter('ir_channel_topics', default_topics)
        self.declare_parameter('pose_topic', 'vision_pose_cov_bridged')
        self.declare_parameter('measurement_topic', 'ir_measurement')
        self.declare_parameter('trigger_service_name', 'trigger_ir_measurement')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('fusion_window_s', 0.2)

        channel_topics = list(self.get_parameter('ir_channel_topics').value)
        if len(channel_topics) != NUM_CHANNELS:
            raise ValueError(
                f'ir_channel_topics must have {NUM_CHANNELS} entries, '
                f'got {len(channel_topics)}.')
        self._measurement_topic = str(self.get_parameter('measurement_topic').value)
        self._trigger_service_name = str(
            self.get_parameter('trigger_service_name').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._fusion_window_s = float(self.get_parameter('fusion_window_s').value)

        self._current_pose = None
        self._channel_buffers = [deque() for _ in range(NUM_CHANNELS)]

        self._channel_subs = [
            self.create_subscription(
                Float32, channel_topics[0], lambda msg: self._on_channel(0, msg), 10),
            self.create_subscription(
                Float32, channel_topics[1], lambda msg: self._on_channel(1, msg), 10),
            self.create_subscription(
                Float32, channel_topics[2], lambda msg: self._on_channel(2, msg), 10),
            self.create_subscription(
                Float32, channel_topics[3], lambda msg: self._on_channel(3, msg), 10),
            self.create_subscription(
                Float32, channel_topics[4], lambda msg: self._on_channel(4, msg), 10),
            self.create_subscription(
                Float32, channel_topics[5], lambda msg: self._on_channel(5, msg), 10),
            self.create_subscription(
                Float32, channel_topics[6], lambda msg: self._on_channel(6, msg), 10),
            self.create_subscription(
                Float32, channel_topics[7], lambda msg: self._on_channel(7, msg), 10),
            self.create_subscription(
                Float32, channel_topics[8], lambda msg: self._on_channel(8, msg), 10),
        ]

        pose_topic = str(self.get_parameter('pose_topic').value)
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._on_pose, 10)

        self._measurement_pub = self.create_publisher(
            ScalarMeasurement, self._measurement_topic, 10)

        self._trigger_srv = self.create_service(
            Trigger, self._trigger_service_name, self._handle_trigger)

    def _on_channel(self, index: int, msg: Float32):
        self._channel_buffers[index].append((self.get_clock().now(), msg.data))

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        self._current_pose = msg.pose.pose

    def _drop_stale(self, buffer: deque):
        cutoff = self.get_clock().now() - rclpy.time.Duration(
            seconds=self._fusion_window_s)
        while buffer and buffer[0][0] < cutoff:
            buffer.popleft()

    def _handle_trigger(self, request: Trigger.Request,
                        response: Trigger.Response) -> Trigger.Response:
        del request

        if self._current_pose is None:
            response.success = False
            response.message = 'No vehicle pose received yet.'
            return response

        channel_means = []
        for i, buffer in enumerate(self._channel_buffers):
            self._drop_stale(buffer)
            if not buffer:
                response.success = False
                response.message = f'No recent data on IR channel {i}.'
                return response
            values = [value for _, value in buffer]
            channel_means.append(sum(values) / len(values))

        measurement = ScalarMeasurement()
        measurement.header.stamp = self.get_clock().now().to_msg()
        measurement.header.frame_id = self._frame_id
        measurement.pose = self._current_pose
        measurement.value = fuse_ir_channels(channel_means)
        measurement.clipped = False
        self._measurement_pub.publish(measurement)

        response.success = True
        response.message = 'Published one fused IR measurement.'
        return response


def main():
    rclpy.init()
    node = IrMeasurementBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
