#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, Polygon
from ipp_interfaces.msg import PolygonsStamped

DEPTH = -0.5
PUBLISH_RATE_MS = 100


class ObstaclePublisherNode(Node):
    def __init__(self):
        super().__init__("obstacle_publisher")
        self.declare_parameter("scenario", "")

        self.obstacles_ = []
        scenario = self.get_parameter("scenario").get_parameter_value().string_value
        if scenario:
            file_path = self._get_scenario_file_path(scenario)
            self._load_obstacles(file_path)
            self.get_logger().info(
                f"Loaded {len(self.obstacles_)} obstacle(s) from scenario '{scenario}'."
            )
        else:
            self.get_logger().info("No scenario provided. Running without obstacles.")

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.obstacles_pub_ = self.create_publisher(PolygonsStamped, "obstacles", qos)
        self.timer_ = self.create_timer(PUBLISH_RATE_MS / 1000.0, self._on_timer)

    def _get_scenario_file_path(self, scenario_name):
        share_dir = get_package_share_directory("ipp")
        return f"{share_dir}/config/{scenario_name}.yaml"

    def _load_obstacles(self, file_path):
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
        for obstacle in data["obstacles"]:
            polygon = Polygon()
            for corner in obstacle["corners"]:
                p = Point32(x=float(corner[0]), y=float(corner[1]), z=DEPTH)
                polygon.points.append(p)
            polygon.points.append(polygon.points[0])
            self.obstacles_.append(polygon)

    def _on_timer(self):
        msg = PolygonsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.polygons = self.obstacles_
        self.obstacles_pub_.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ObstaclePublisherNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
