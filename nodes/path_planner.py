#!/usr/bin/env python3

import math
import sys
import threading
from enum import Enum, auto
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ipp_interfaces.srv import SetPath
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

# Tells Python to look for rig_tree.py in the same directory as this file.
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from rig_tree import RIGTree


class State(Enum):
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()


def occupancy_grid_to_matrix(grid: OccupancyGrid):
    """Converts a flat OccupancyGrid message into a 2D numpy array (rows=y, cols=x)."""
    data = np.array(grid.data, dtype=np.uint8)
    return data.reshape(grid.info.height, grid.info.width)


class PathPlanner(Node):
    """Receding-horizon IPP node. Plans with RIG-tree and replans when the path end is reached."""

    def __init__(self):
        super().__init__(node_name='path_planner')
        self.state = State.IDLE

        self._declare_parameters()

        self.cell_size: float = None
        self.occupancy_grid: OccupancyGrid = None
        self.occupancy_matrix: np.ndarray = None
        self.current_pose: Pose = None

        self._lock = threading.Lock()
        self._pending_path: list = None
        self._new_path_ready: bool = False
        self._path_end: np.ndarray = None
        self._planning_active: bool = False
        self._last_replan_time: float = 0.0

        self.init_path_marker()
        self.init_publishers()
        self.init_subscribers()
        self.init_clients()
        self.init_services()
        self._control_timer = self.create_timer(0.1, self.control_timer_cb)

    # ------------------------------------------------------------------
    # Algorithm parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        # Planning horizon (m) per cycle, B in the paper (Section 4.1).
        # Nodes with path length > B enter the closed set and represent complete paths.
        # Larger value means longer path per cycle and less frequent replanning.
        self.declare_parameter('planning_horizon', 0.5)

        # Maximum edge length (m) between two tree nodes, Delta in the paper.
        # B/Delta gives the maximum tree depth (roughly 2 to 3 levels at 0.5/0.2).
        self.declare_parameter('delta', 0.2)

        # Neighbourhood radius (m) for the Near() search, R in the paper.
        # The paper leaves R open ("heuristically set", Section 4).
        # Larger value means more connections per iteration but slower planning.
        self.declare_parameter('near_radius', 0.3)

        # Planning time per cycle (s). RIG-tree is an anytime algorithm
        # ("while not terminated", Algorithm 2) and runs as long as the operator allows.
        # Should be shorter than the travel time for one planning horizon so the next
        # path is ready before the robot reaches the path end. If it is longer, the
        # robot waits at the path end and planning_time_s effectively becomes the
        # replanning frequency.
        self.declare_parameter('planning_time_s', 0.5)

        # Distance (m) to the last waypoint that triggers a new planning cycle.
        # The timer checks this condition: it is the paper-conform "budget expended"
        # signal (Section 4.1). Should be smaller than look_ahead_distance of
        # path_follower (0.3 m).
        self.declare_parameter('goal_tolerance', 0.15)

        # Start position for the move_to_start service.
        self.declare_parameter('robot_z', -0.5)
        self.declare_parameter('start_x', 0.45)
        self.declare_parameter('start_y', 0.45)
        self.declare_parameter('start_yaw', 0.0)

        self.declare_parameter('execution_horizon_m', 0.2)

        # Time-based safety fallback, not paper-conform.
        # Forces replanning if the robot never reaches the path end (e.g. stuck).
        self.declare_parameter('replan_timeout_s', 20.0)

    # ------------------------------------------------------------------
    # ROS connections setup
    # ------------------------------------------------------------------

    def init_publishers(self):
        self.path_marker_pub = self.create_publisher(Marker, '~/marker', 1)

    def init_subscribers(self):
        self.grid_map_sub = self.create_subscription(OccupancyGrid,
                                                     'occupancy_grid',
                                                     self.on_occupancy_grid, 1)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                 'vision_pose_cov',
                                                 self.on_pose, 1)

    def init_services(self):
        self.start_service = self.create_service(Trigger, '~/start',
                                                 self.serve_start)
        self.stop_service = self.create_service(Trigger, '~/stop',
                                                self.serve_stop)
        self.move_to_start_service = self.create_service(
            Trigger, '~/move_to_start', self.serve_move_to_start)

    def init_clients(self):
        cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.set_path_client = self.create_client(SetPath,
                                                  'path_follower/set_path',
                                                  callback_group=cb_group)
        self.path_finished_client = self.create_client(
            Trigger, 'path_follower/path_finished', callback_group=cb_group)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def on_occupancy_grid(self, msg: OccupancyGrid):
        self.occupancy_grid = msg
        self.occupancy_matrix = occupancy_grid_to_matrix(msg)
        self.cell_size = msg.info.resolution

    def on_pose(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def serve_start(self, _request, response):
        self.get_logger().info(
            'Start requested, beginning receding-horizon IPP.')
        self.state = State.PLANNING
        self.trigger_replan()
        response.success = True
        return response

    def serve_stop(self, _request, response):
        self.get_logger().info('Stop requested.')
        response.success = self.do_stop()
        return response

    def do_stop(self):
        self.state = State.IDLE
        with self._lock:
            self._path_end = None
            self._pending_path = None
            self._new_path_ready = False
        self.publish_path_marker([])
        try:
            future = self.path_finished_client.call_async(Trigger.Request())
            return future is not None
        except Exception:
            return False

    def serve_move_to_start(self, _request, response):
        if self.state == State.IDLE and self.current_pose is not None:
            self.get_logger().info('Move to start requested.')
            response.success = self.do_move_to_start()
        else:
            self.get_logger().warn(
                f'move_to_start ignored: state={self.state.name}, '
                f'pose_available={self.current_pose is not None}')
            response.success = False
        return response

    def do_move_to_start(self):
        z = self.get_parameter('robot_z').value
        gx = self.get_parameter('start_x').value
        gy = self.get_parameter('start_y').value

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        dist = math.hypot(gx - cx, gy - cy)
        n = max(2,
                math.ceil(dist / 0.3) +
                1)  # waypoints at 0.3 m spacing, at least 2

        o = self.current_pose.orientation
        current_yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        start_yaw = self.get_parameter('start_yaw').value

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Straight-line waypoints at 0.3 m spacing
        # yaw gradually rotates to yaw at start position (interpolation)
        xs = np.linspace(cx, gx, n)
        ys = np.linspace(cy, gy, n)
        yaws = np.linspace(current_yaw, start_yaw, n)

        poses = []
        for x, y, yaw in zip(xs, ys, yaws):
            q = quaternion_from_euler(0.0, 0.0, float(yaw))
            poses.append(
                PoseStamped(header=header,
                            pose=Pose(position=Point(x=float(x),
                                                     y=float(y),
                                                     z=z),
                                      orientation=Quaternion(x=q[0],
                                                             y=q[1],
                                                             z=q[2],
                                                             w=q[3]))))

        ros_path = Path()
        ros_path.poses = poses

        req = SetPath.Request()
        req.path = ros_path
        future = self.set_path_client.call_async(req)
        return future is not None

    # ------------------------------------------------------------------
    # Receding-horizon loop
    # ------------------------------------------------------------------

    def trigger_replan(self):
        with self._lock:
            if self._planning_active:
                return
            if self.current_pose is None or self.occupancy_matrix is None:
                missing = []
                if self.current_pose is None:
                    missing.append('vision_pose_cov')
                if self.occupancy_matrix is None:
                    missing.append('occupancy_grid')
                self.get_logger().warn(
                    f'Cannot plan: waiting for {", ".join(missing)}.',
                    throttle_duration_sec=2.0)
                return
            self._planning_active = True

        threading.Thread(target=self.run_planning, daemon=True).start()

    def run_planning(self):
        pose = self.current_pose
        x_start = np.array([pose.position.x, pose.position.y])
        grid = self.occupancy_matrix.copy()

        self.get_logger().info(
            f'Planning from ({x_start[0]:.2f}, {x_start[1]:.2f}), '
            f'horizon={self.get_parameter("planning_horizon").value} m ...')

        tree = RIGTree(
            delta=self.get_parameter('delta').value,
            planning_horizon=self.get_parameter('planning_horizon').value,
            x_start=x_start,
            R=self.get_parameter('near_radius').value,
            grid=grid,
            resolution=self.cell_size,
            info_fn=self.make_info_fn(),
        )
        tree.run(planning_time_s=self.get_parameter('planning_time_s').value)
        path = tree.best_path()

        with self._lock:
            self._pending_path = path
            self._new_path_ready = True
            self._planning_active = False

    def make_info_fn(self):
        """Returns the information function. Replace with GP variance lookup when available."""
        return None  # placeholder: RIGTree counts +1.0 per step (modular)
        # Later: return lambda i_near, x_new: i_near + self._gp_variance_at(x_new)

    def control_timer_cb(self):
        """10 Hz: send finished paths and check the replanning condition.

        The planning thread must not make ROS calls (executor conflict), so it only
        stores the result in _pending_path. This timer runs in the executor context
        and sends the path. Exactly one of the three tasks below is handled per tick.
        """

        # Task 1: send a new path if the planning thread has finished.
        # Lock needed because _pending_path is written by the thread.
        path = None
        with self._lock:
            if self._new_path_ready and self._pending_path:
                path = self._pending_path
                self._new_path_ready = False

        if path:
            if self.state != State.IDLE:
                self.send_path(path)
            return

        # Task 2: retry starting the planning thread.
        # Happens when PLANNING state was set but trigger_replan() returned early
        # because pose or grid were not yet available, or the last path was empty.
        if self.state == State.PLANNING:
            with self._lock:
                active = self._planning_active
            if not active:
                self.trigger_replan()
            return

        if self.state != State.EXECUTING:
            return

        if self.current_pose is None or self._path_end is None:
            return

        # Task 3: check the replanning condition.
        # Primary condition (paper-conform, Section 4.1):
        #   "replanning after the budget increment is expended"
        #   The budget is expended when the robot has reached the path end.
        pos = self.current_pose.position
        dist = math.sqrt((pos.x - self._path_end[0])**2 +
                         (pos.y - self._path_end[1])**2)

        if dist < self.get_parameter('goal_tolerance').value:
            self.get_logger().info('Path end reached, replanning.')
            self.state = State.PLANNING
            self.trigger_replan()
            return

        # Secondary condition (not paper-conform, practical fallback):
        #   Robot is stuck and never gets close enough to goal_tolerance.
        elapsed = self.get_clock().now(
        ).nanoseconds * 1e-9 - self._last_replan_time
        if elapsed > self.get_parameter('replan_timeout_s').value:
            self.get_logger().warn(
                f'Replan timeout ({self.get_parameter("replan_timeout_s").value} s),'
                ' forcing replan.')
            self.state = State.PLANNING
            self.trigger_replan()

    def _truncate_path_to_distance(self, path: list, max_dist: float) -> list:
        """Cuts path short so the robot travels at most max_dist meters along it."""
        if len(path) <= 2:
            return path
        traveled = 0.0
        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            traveled += math.hypot(x1 - x0, y1 -
                                   y0)  # cumulative distance of path segments
            if traveled >= max_dist:
                return path[:i + 1]
        return path

    def send_path(self, path: list):
        """Sends only the execution-horizon slice of the planned path to path_follower."""
        if not path:
            self.get_logger().warn('Empty path from RIG-tree, retrying.')
            self.state = State.PLANNING
            return

        exec_horizon = self.get_parameter('execution_horizon_m').value
        executed_path = self._truncate_path_to_distance(path, exec_horizon)

        ros_path = self.rig_path_to_ros(executed_path)
        req = SetPath.Request()
        req.path = ros_path
        self.set_path_client.call_async(req)

        self._path_end = executed_path[-1]
        self._last_replan_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = State.EXECUTING

        self.publish_path_marker(path)
        self.get_logger().info(
            f'Executing {len(executed_path)} of {len(path)} waypoints '
            f'(execution_horizon={exec_horizon:.2f} m).')

    def rig_path_to_ros(self, path: list) -> Path:
        """Converts a list of [x, y] points into a nav_msgs/Path message."""
        z = self.get_parameter('robot_z').value

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        points_3d = [Point(x=float(xy[0]), y=float(xy[1]), z=z) for xy in path]

        # Yaw points toward the next waypoint; the last point keeps the same direction.
        yaws = [
            math.atan2(path[i + 1][1] - path[i][1], path[i + 1][0] - path[i][0])
            for i in range(len(path) - 1)
        ]
        yaws.append(yaws[-1] if yaws else 0.0)

        orientations = []
        for yaw in yaws:
            q = quaternion_from_euler(0.0, 0.0, yaw)
            orientations.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

        ros_path = Path()
        ros_path.poses = [
            PoseStamped(header=header, pose=Pose(position=pt, orientation=ori))
            for pt, ori in zip(points_3d, orientations)
        ]
        return ros_path

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def init_path_marker(self):
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

    def publish_path_marker(self, path: list):
        """Publishes the planned path as a line strip in RViz."""
        z = self.get_parameter('robot_z').value
        self.path_marker.header.stamp = self.get_clock().now().to_msg()
        self.path_marker.points = [
            Point(x=float(xy[0]), y=float(xy[1]), z=z) for xy in path
        ]
        self.path_marker_pub.publish(self.path_marker)


def main():
    rclpy.init()
    node = PathPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
