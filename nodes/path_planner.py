#!/usr/bin/env python3

from enum import Enum, auto

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scenario_msgs.srv import SetPath
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class State(Enum):
    UNSET = auto()
    IDLE = auto()


def occupancy_grid_to_matrix(grid: OccupancyGrid):
    """Convert a flat OccupancyGrid message into a 2D numpy array (rows=y, cols=x)."""
    data = np.array(grid.data, dtype=np.uint8)
    data = data.reshape(grid.info.height, grid.info.width)
    return data


def world_to_matrix(x, y, grid_size):
    """Convert world coordinates (m) to [col, row] grid indices."""
    return [round(x / grid_size), round(y / grid_size)]


class PathPlanner(Node):
    """Path planner node. Subscribes to the occupancy grid and exposes start/stop services."""

    def __init__(self):
        super().__init__(node_name='path_planner')
        self.state = State.UNSET
        self.cell_size = 0.2
        self.path_marker: Marker
        self.init_path_marker()
        self.path_marker_pub = self.create_publisher(Marker, '~/marker', 1)
        self.occupancy_grid: OccupancyGrid = None
        self.occupancy_matrix: np.ndarray = None
        self.init_clients()
        self.init_services()
        self.grid_map_sub = self.create_subscription(OccupancyGrid,
                                                     'occupancy_grid',
                                                     self.on_occupancy_grid, 1)

    def init_services(self):
        """Register ROS2 services: start and stop."""
        self.start_service = self.create_service(Trigger, '~/start',
                                                 self.serve_start)
        self.stop_service = self.create_service(Trigger, '~/stop',
                                                self.serve_stop)

    def init_clients(self):
        """Create service clients for communicating with path_follower."""
        cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.set_path_client = self.create_client(SetPath,
                                                  'path_follower/set_path',
                                                  callback_group=cb_group)
        self.path_finished_client = self.create_client(
            Trigger, 'path_follower/path_finished', callback_group=cb_group)

    def has_collisions(self, points_2d):
        """Return indices of points_2d ([col, row] pairs) that hit an obstacle (cell >= 50)."""
        if not self.occupancy_grid:
            return []
        collision_indices = [
            i for i, p in enumerate(points_2d)
            if self.occupancy_matrix[p[1], p[0]] >= 50
        ]
        return collision_indices

    def serve_start(self, request, response):
        """Trigger service: transition to IDLE and signal readiness to begin the IPP mission."""
        self.get_logger().info('Start requested.')
        self.state = State.IDLE
        response.success = True
        return response

    def serve_stop(self, request, response):
        """Trigger service: stop the current mission and transition to IDLE."""
        if self.state != State.IDLE:
            self.get_logger().info('Asked to stop. Going to idle mode.')
        response.success = self.do_stop()
        return response

    def do_stop(self):
        """Tell path_follower to stop. Returns True if confirmed."""
        self.state = State.IDLE
        if self.path_finished_client.call(Trigger.Request()).success:
            self.state = State.IDLE
            return True
        return False

    def on_occupancy_grid(self, msg: OccupancyGrid):
        """Store the latest inflated occupancy grid and update cell_size if resolution changed."""
        self.occupancy_grid = msg
        self.occupancy_matrix = occupancy_grid_to_matrix(self.occupancy_grid)
        if msg.info.resolution != self.cell_size:
            self.get_logger().info('Cell size changed.')
            self.cell_size = msg.info.resolution

    def init_path_marker(self):
        """Initialise the reusable Marker message used to visualise paths in RViz."""
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = 'path'
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.header.frame_id = 'map'
        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.scale.x = 0.02
        msg.scale.y = 0.02
        msg.scale.z = 0.02
        self.path_marker = msg

    def set_new_path(self, path):
        """Asynchronously send a nav_msgs/Path to path_follower. Returns False if path is empty."""
        request = SetPath.Request()
        if not path:
            return False
        request.path = path
        self.set_new_path_future = self.set_path_client.call_async(request)
        return True


def main():
    rclpy.init()
    node = PathPlanner()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
