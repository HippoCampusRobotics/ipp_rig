#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Polygon, Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from ipp.msg import PolygonsStamped

OCCUPIED = 100
DEPTH = -0.5


class MapperNode(Node):

    def __init__(self):
        super().__init__(node_name='mapper')

        self.tank_size_x = None
        self.tank_size_y = None
        self.cell_resolution = None
        self.init_params()

        self.num_cells_x = int(np.round(self.tank_size_x /
                                        self.cell_resolution))
        self.num_cells_y = int(np.round(self.tank_size_y /
                                        self.cell_resolution))

        self.map_meta_data = self.get_map_meta_data()

        # Two maps are maintained:
        # 1. map: inflated obstacles that account for the robot size
        # 2. map_uninflated: obstacles at their original size (for debugging)

        # x is columns, y is rows, so the array shape is (y, x)
        empty_map = np.zeros((self.num_cells_y, self.num_cells_x), dtype='int8')
        self.map = self.get_tank_walls(empty_map)
        self.map_uninflated = np.copy(self.map)

        self.get_logger().info(
            f'Cell resolution: {self.cell_resolution} m/cell, '
            f'{self.num_cells_x} cells in x, {self.num_cells_y} cells in y.')

        self.map_pub = self.create_publisher(msg_type=OccupancyGrid,
                                             topic='occupancy_grid',
                                             qos_profile=1)

        self.map_pub_debug = self.create_publisher(
            msg_type=OccupancyGrid,
            topic='/occupancy_grid_uninflated',
            qos_profile=1)

        self.obstacle_sub = self.create_subscription(msg_type=PolygonsStamped,
                                                     topic='obstacles',
                                                     callback=self.on_obstacles,
                                                     qos_profile=1)

        # Publish the map at 1 Hz so late subscribers (e.g. path_planner) always
        # receive the current map even when no obstacle updates have arrived.
        self.create_timer(1.0, self.publish_timer_cb)

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tank_size_x', rclpy.Parameter.Type.DOUBLE),
                ('tank_size_y', rclpy.Parameter.Type.DOUBLE),
                ('cell_resolution', rclpy.Parameter.Type.DOUBLE),
            ])
        param = self.get_parameter('tank_size_x')
        self.get_logger().info(f'{param.name}={param.value}')
        self.tank_size_x = param.value
        param = self.get_parameter('tank_size_y')
        self.get_logger().info(f'{param.name}={param.value}')
        self.tank_size_y = param.value
        param = self.get_parameter('cell_resolution')
        self.get_logger().info(f'{param.name}={param.value}')
        self.cell_resolution = param.value

    def publish_timer_cb(self):
        self.publish_map(np.copy(self.map), self.map_pub)

    def on_obstacles(self, msg: PolygonsStamped):
        num_obstacles = len(msg.polygons)
        if not num_obstacles:
            return

        for obstacle in msg.polygons:
            self.map_uninflated = self.include_obstacle_in_map(
                obstacle, np.copy(self.map_uninflated))

        self.map = self.inflate_obstacles(np.copy(self.map_uninflated))

        self.publish_map(np.copy(self.map), self.map_pub)
        self.publish_map(np.copy(self.map_uninflated), self.map_pub_debug)

    def include_obstacle_in_map(self, obstacle: Polygon,
                                cells: np.ndarray) -> np.ndarray:
        num_points = len(obstacle.points)
        obstacle_cells = np.zeros((num_points, 2), dtype='uint32')

        for index, point in enumerate(obstacle.points):
            obstacle_cells[index,
                           0] = int(round(point.x / self.cell_resolution))
            obstacle_cells[index,
                           1] = int(round(point.y / self.cell_resolution))

        # Draw the obstacle as a filled polygon into the grid
        img = cells.astype(dtype='uint8')
        img_obstacles = cv2.fillPoly(img,
                                     pts=np.int32([obstacle_cells]),
                                     color=(OCCUPIED))

        return img_obstacles.astype(dtype='int8')

    def inflate_obstacles(self, cells: np.ndarray) -> np.ndarray:
        # Inflate obstacles by approximately 30 cm so the planner keeps
        # the robot away from walls and obstacles
        img = cells.astype(dtype='uint8')
        dilatation_size = int(np.round(0.3 / self.cell_resolution))

        # Elliptical kernel for uniform inflation in all directions
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (dilatation_size, dilatation_size))
        img_dilated = cv2.dilate(img, kernel, iterations=1)

        return img_dilated.astype(dtype='int8')

    def get_tank_walls(self, cells: np.ndarray) -> np.ndarray:
        # Mark all borders of the grid as occupied
        cells[:, 0] = OCCUPIED
        cells[:, -1] = OCCUPIED
        cells[0, :] = OCCUPIED
        cells[-1, :] = OCCUPIED
        return cells

    def publish_map(self, data: np.ndarray, pub):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self.map_meta_data
        msg.data = np.copy(data).flatten()
        pub.publish(msg)

    def get_map_meta_data(self) -> MapMetaData:
        meta_data = MapMetaData()
        meta_data.map_load_time = self.get_clock().now().to_msg()
        meta_data.resolution = self.cell_resolution
        meta_data.width = self.num_cells_x
        meta_data.height = self.num_cells_y
        # Origin at (0, 0, operating depth), same orientation as the map frame
        meta_data.origin = Pose()
        meta_data.origin.position.z = DEPTH
        return meta_data


def main():
    rclpy.init()
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
