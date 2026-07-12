#!/usr/bin/env python3

import csv
import datetime
import math
import os
import random
import sys
import threading
from enum import Enum, auto
from math import pi

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ipp_interfaces.srv import SetPath
from scalar_field_interfaces.srv import QueryScalarFieldBeliefCovariance
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

# Tells Python to look for rig_tree.py/lawnmower.py in the same directory as this file.
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from rig_tree import RIGTree
from lawnmower import generate_lawnmower_path

# nodes/path_planner.py -> ipp -> src -> workspace root. Both packages are
# symlink-installed, so __file__ resolves to this source path even when
# launched from the colcon install space.
_WORKSPACE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))


class State(Enum):
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()


def occupancy_grid_to_matrix(grid: OccupancyGrid):
    """Converts a flat OccupancyGrid message into a 2D numpy array (rows=y, cols=x)."""
    data = np.array(grid.data, dtype=np.uint8)
    return data.reshape(grid.info.height, grid.info.width)


class PathPlanner(Node):
    """Receding-horizon IPP node. Plans with RIG-tree and replans when the path end is reached.

    Set the 'planner_type' parameter to 'lawnmower' to instead run a
    non-informative lawnmover coverage planner as a baseline: it is computed
    once and sent in full (no replanning), see lawnmower.py.

    Set 'planner_type' to 'random' to run the same RIG-tree receding-horizon
    loop (same control, obstacle avoidance, horizon/replanning behaviour) but
    without any variance information: candidate nodes get a uniform info
    increment instead of a GP covariance query, so the path is effectively a
    random walk within the tank.
    """

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
        self._planning_cancel_event: threading.Event = None
        self._last_replan_time: float = 0.0

        # Debugging Variables ----------
        self._planning_cycle: int = 0
        self._info_call_idx: int = 0
        self._init_debug_logs()
        # ------------------------------

        self.init_path_marker()
        self.init_publishers()
        self.init_subscribers()
        self.init_clients()
        self.init_services()
        self._control_timer = self.create_timer(0.1, self.control_timer_cb)
        self._measurement_timer = self.create_timer(
            self.get_parameter('measurement_period_s').value,
            self._trigger_measurement)

    # ------------------------------------------------------------------
    # Algorithm parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        # 'rig' (RIG-tree), 'lawnmower' (non-informative coverage baseline),
        # or 'random' (RIG-tree without variance information, i.e. a random walk).
        self.declare_parameter('planner_type', 'rig')

        # Seeds Python's random module (used by RIGTree.Sample()) once at
        # mission start. -1 (default) leaves the RNG unseeded. Set the same
        # non-negative seed across a rig/random comparison run to make the
        # planners see the same sequence of sample draws for a fair,
        # reproducible comparison.
        self.declare_parameter('rng_seed', -1)

        # Distance (m) between sweep lanes. Only used when planner_type='lawnmower'.
        self.declare_parameter('lane_spacing', 0.27)

        # Planning horizon (m) per cycle, B in the paper (Section 4.1).
        # Nodes with path length > B enter the closed set and represent complete paths.
        # Also used as the execution distance: the planned path is truncated
        # to this length before being sent to path_follower.
        # Larger value means longer path per cycle and less frequent replanning.
        self.declare_parameter('planning_horizon', 5.0)

        # Maximum edge length (m) between two tree nodes, Delta in the paper.
        # B/Delta gives the maximum tree depth (roughly 2 to 3 levels at 0.5/0.2).
        self.declare_parameter('delta', 0.5)

        # Neighbourhood radius (m) for the Near() search, R in the paper.
        # The paper leaves R open ("heuristically set", Section 4).
        # Larger value means more connections per iteration but slower planning.
        self.declare_parameter('near_radius', 0.5)

        # Planning time per cycle (s). RIG-tree is an anytime algorithm
        # ("while not terminated", Algorithm 2) and runs as long as the operator allows.
        # Should be shorter than the travel time for one planning horizon so the next
        # path is ready before the robot reaches the path end. If it is longer, the
        # robot waits at the path end and planning_time_s effectively becomes the
        # replanning frequency.
        self.declare_parameter('planning_time_s', 8.0)

        # Distance (m) to the last waypoint that triggers a new planning cycle.
        # The timer checks this condition: it is the paper-conform "budget expended"
        # signal (Section 4.1). Should be smaller than look_ahead_distance of
        # path_follower (0.3 m).
        self.declare_parameter('goal_tolerance', 0.15)

        # Start position for the move_to_start service.
        self.declare_parameter('robot_z', -0.5)
        self.declare_parameter('start_x', 0.45)
        self.declare_parameter('start_y', 0.45)
        self.declare_parameter('start_yaw', pi / 2.0)

        # Time-based safety fallback, not paper-conform.
        # Forces replanning if the robot never reaches the path end (e.g. stuck).
        self.declare_parameter('replan_timeout_s', 30.0)

        # Period (s) between IR measurement triggers while executing a path.
        # 0.2 s = 5 Hz.
        self.declare_parameter('measurement_period_s', 1.0)

        # Points per axis for the fixed grid used to score how informative a
        # candidate path is (grid size is info_grid_resolution^2). Bigger
        # grid = more accurate info score but slower, since every RIG-tree
        # iteration queries all grid points together with the candidates.
        self.declare_parameter('info_grid_resolution', 26)

    # ------------------------------------------------------------------
    # Debug logging (info-increment / measurement-count / tree checks)
    # ------------------------------------------------------------------

    def _init_debug_logs(self):
        run_ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.join(_WORKSPACE_ROOT, 'debug_logs', 'path_planner',
                               run_ts)
        os.makedirs(log_dir, exist_ok=True)

        self._info_csv = csv.writer(
            open(os.path.join(log_dir, 'info_increments.csv'),
                 'w',
                 newline='',
                 buffering=1))
        self._info_csv.writerow(
            ['cycle', 'call_idx', 'n_candidates', 'min', 'max', 'mean', 'std'])

        self._meas_csv = csv.writer(
            open(os.path.join(log_dir, 'measurement_count.csv'),
                 'w',
                 newline='',
                 buffering=1))
        self._meas_csv.writerow(
            ['cycle', 'ros_time_s', 'x_start_x', 'x_start_y'])

        self._tree_csv = csv.writer(
            open(os.path.join(log_dir, 'tree_stats.csv'),
                 'w',
                 newline='',
                 buffering=1))
        self._tree_csv.writerow([
            'cycle', 'n_nodes', 'n_closed', 'best_info', 'closed_info_min',
            'closed_info_max', 'closed_info_mean', 'closed_info_std'
        ])

        self._path_csv = csv.writer(
            open(os.path.join(log_dir, 'best_path.csv'),
                 'w',
                 newline='',
                 buffering=1))
        self._path_csv.writerow(['cycle', 'point_idx', 'x', 'y'])

        self.get_logger().info(f'Debug logs: {log_dir}')

    def _log_info_increments(self, arr: np.ndarray):
        self._info_call_idx += 1
        self._info_csv.writerow([
            self._planning_cycle, self._info_call_idx,
            len(arr),
            float(arr.min()),
            float(arr.max()),
            float(arr.mean()),
            float(arr.std())
        ])

    def _log_measurement_cycle(self, x_start: np.ndarray):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._meas_csv.writerow(
            [self._planning_cycle, now,
             float(x_start[0]),
             float(x_start[1])])

    def _log_tree_stats(self, stats: dict):
        self._tree_csv.writerow([
            self._planning_cycle, stats['nodes'], stats['closed'],
            stats.get('best_info'),
            stats.get('closed_info_min'),
            stats.get('closed_info_max'),
            stats.get('closed_info_mean'),
            stats.get('closed_info_std')
        ])

    def _log_best_path(self, path: list):
        for i, point in enumerate(path):
            self._path_csv.writerow(
                [self._planning_cycle, i,
                 float(point[0]),
                 float(point[1])])

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

        belief_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.belief_covariance_client = self.create_client(
            QueryScalarFieldBeliefCovariance,
            'query_scalar_field_belief_covariance',
            callback_group=belief_cb_group,
        )

        measurement_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup(
        )
        self.measurement_client = self.create_client(
            Trigger,
            'trigger_ir_measurement',
            callback_group=measurement_cb_group,
        )

        reset_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.reset_belief_client = self.create_client(
            Trigger,
            'reset_scalar_field_belief',
            callback_group=reset_cb_group,
        )

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
            f'Start requested, planner_type={self.get_parameter("planner_type").value}.'
        )
        rng_seed = self.get_parameter('rng_seed').value
        if rng_seed >= 0:
            random.seed(rng_seed)
            self.get_logger().info(f'Seeded RNG with rng_seed={rng_seed}.')
        # One measurement at the start position before the first planning
        # cycle, so the very first cycle already scores candidates against
        # a non-empty belief instead of the bare prior.
        if self.measurement_client.service_is_ready():
            self.measurement_client.call_async(Trigger.Request())
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
            cancel_event = self._planning_cancel_event
        if cancel_event is not None:
            cancel_event.set()
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
        if self.reset_belief_client.service_is_ready():
            self.reset_belief_client.call_async(Trigger.Request())
            self.get_logger().info('Belief reset.')
        z = self.get_parameter('robot_z').value
        gx = self.get_parameter('start_x').value
        gy = self.get_parameter('start_y').value

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        dist = math.hypot(gx - cx, gy - cy)
        n = max(2,
                math.ceil(dist / 0.3) +
                1)  # waypoints at 0.3 m spacing, at least 2

        start_yaw = self.get_parameter('start_yaw').value

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Straight-line waypoints at 0.3 m spacing
        xs = np.linspace(cx, gx, n)
        ys = np.linspace(cy, gy, n)

        q = quaternion_from_euler(0.0, 0.0, float(start_yaw))
        last_orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        poses = []
        for i, (x, y) in enumerate(zip(xs, ys)):
            orientation = last_orientation if i == n - 1 else Quaternion()
            poses.append(
                PoseStamped(header=header,
                            pose=Pose(position=Point(x=float(x),
                                                     y=float(y),
                                                     z=z),
                                      orientation=orientation)))

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
            cancel_event = threading.Event()
            self._planning_cancel_event = cancel_event

        if self.get_parameter('planner_type').value == 'lawnmower':
            # Cheap geometry, no need for a background thread.
            self.run_lawnmower_planning()
        else:
            threading.Thread(target=self.run_planning,
                             args=(cancel_event, ),
                             daemon=True).start()

    def run_lawnmower_planning(self):
        x_start = np.array([
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
        ])
        height, width = self.occupancy_matrix.shape

        path = generate_lawnmower_path(
            x_start=x_start,
            x_max=width * self.cell_size,
            y_max=height * self.cell_size,
            lane_spacing=self.get_parameter('lane_spacing').value,
            waypoint_spacing=self.get_parameter('delta').value,
        )
        self.get_logger().info(
            f'Lawnmower coverage path with {len(path)} waypoints.')

        with self._lock:
            self._pending_path = path
            self._new_path_ready = True
            self._planning_active = False

    def run_planning(self, cancel_event: threading.Event):
        self._planning_cycle += 1
        # The robot is stationary (holding at the previous path's end) for
        # the whole PLANNING phase, so the live pose is exactly x_start.
        pose = self.current_pose
        x_start = np.array([pose.position.x, pose.position.y])
        planning_time_s = self.get_parameter('planning_time_s').value
        grid = self.occupancy_matrix.copy()

        self.get_logger().info(
            f'Planning from ({x_start[0]:.2f}, {x_start[1]:.2f}), '
            f'horizon={self.get_parameter("planning_horizon").value} m ...')

        self._log_measurement_cycle(x_start)

        # Grid for scoring information this cycle: built once here and
        # reused for every candidate the tree scores.
        info_grid_points = self._build_info_grid()

        is_random = self.get_parameter('planner_type').value == 'random'
        tree = RIGTree(
            delta=self.get_parameter('delta').value,
            planning_horizon=self.get_parameter('planning_horizon').value,
            x_start=x_start,
            R=self.get_parameter('near_radius').value,
            grid=grid,
            resolution=self.cell_size,
            batch_info_fn=None
            if is_random else self.make_batch_info_fn(info_grid_points),
        )
        stats = tree.run(planning_time_s=planning_time_s,
                         cancel_event=cancel_event)
        self.get_logger().info(
            f'Planning done ({stats["stopped_reason"]}): {stats["iterations"]} iterations '
            f'in {stats["elapsed_s"]:.2f} s, '
            f'{stats["nodes_added"]} nodes added ({stats["nodes"]} total, {stats["closed"]} closed), '
            f'{stats["gp_calls"]} belief covariance calls')
        self._log_tree_stats(stats)
        path = tree.best_path()
        self._log_best_path(path)

        with self._lock:
            self._pending_path = path
            self._new_path_ready = True
            self._planning_active = False

    def _trigger_measurement(self):
        if self.state != State.EXECUTING:
            return
        if self.measurement_client.service_is_ready():
            self.measurement_client.call_async(Trigger.Request())

    def _build_info_grid(self) -> list:
        """Builds the fixed grid of points used to score how informative a
        path is. Spread evenly over the whole tank, independent of any
        path. Rebuilt once at the start of each planning cycle in
        run_planning() and then reused for every candidate scored in that
        cycle.
        """
        n = self.get_parameter('info_grid_resolution').value
        height, width = self.occupancy_matrix.shape
        x_max = width * self.cell_size
        y_max = height * self.cell_size
        xs = np.linspace(0.0, x_max, n)
        ys = np.linspace(0.0, y_max, n)
        xx, yy = np.meshgrid(xs, ys)
        return [np.array([x, y]) for x, y in zip(xx.ravel(), yy.ravel())]

    def make_batch_info_fn(self, grid_points: list):
        """Returns a function that scores a batch of candidate paths in one
        belief query, giving each one an information value that gets added
        to its parent's value in rig_tree.run(). Returns None if the belief
        service isn't ready, in which case rig_tree falls back to giving
        every step a value of 1.

        grid_points is the fixed grid built in _build_info_grid: every
        candidate in this planning cycle is compared against the same grid,
        so a path's score is how much uncertainty it removes from the whole
        tank, not just compared to its own ancestors.
        """
        if not self.belief_covariance_client.service_is_ready():
            self.get_logger().warn(
                'Belief covariance service not ready, using uniform information.'
            )
            return None

        n_grid = len(grid_points)
        # Grid queries are identical on every call this cycle, so build
        # them once and only append the (cheap, per-call) path-point
        # queries below.
        grid_query_poses = []
        for pos in grid_points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(pos[0])
            ps.pose.position.y = float(pos[1])
            grid_query_poses.append(ps)

        def batch_info_fn(paths: list) -> list:
            # Candidates often share earlier points on their path (e.g. two
            # candidates branching off the same point), so give every point
            # a number and only query the belief service about each once.
            query_points = []
            index_of = {}

            def get_index(pos):
                # Round to avoid floating-point noise creating a duplicate
                # entry for what is really the same point.
                key = (round(float(pos[0]), 9), round(float(pos[1]), 9))
                idx = index_of.get(key)
                if idx is None:
                    idx = len(query_points)
                    index_of[key] = idx
                    query_points.append(pos)
                return idx

            # Each candidate's path, translated into point numbers (0-based
            # within query_points, i.e. excluding the grid block below).
            candidate_indices = [[get_index(p) for p in path] for path in paths]

            request = QueryScalarFieldBeliefCovariance.Request()
            request.queries.extend(grid_query_poses)
            for pos in query_points:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = float(pos[0])
                ps.pose.position.y = float(pos[1])
                request.queries.append(ps)

            # Do service call on planning thread
            event = threading.Event()
            future = self.belief_covariance_client.call_async(request)
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout=1.0):
                return [1.0] * len(paths)  # Fallback, just put 1 for every path
            result = future.result()
            n = n_grid + len(query_points)
            if (result is None or not result.success
                    or len(result.covariance_matrix) != n * n):
                return [1.0] * len(paths)  # Fallback, just put 1 for every path

            # reshape flat list that the ROS2 service sends into the proper matrix
            cov = np.array(result.covariance_matrix).reshape(n, n)
            # Covariance between grid points and path points, and between
            # path points themselves. We don't need the grid-grid block: it's
            # the same for every candidate in this call, so it cancels out of
            # the increment below.
            c_g_all = cov[:n_grid, n_grid:]  # (M, K)
            c_xx_all = cov[n_grid:, n_grid:]  # (K, K)
            # GP likelihood noise (sigma_n^2), owned by the belief model, not
            # duplicated here: it can change if the belief refits its
            # hyperparameters, so it's read fresh from the service response
            # every call instead of being a planner-side constant.
            sigma_n_sq = result.measurement_noise_variance

            def trace_reduction(idx: list) -> float:
                """How much total grid uncertainty is removed by knowing the
                points at idx, on top of the real measurements already in
                the belief. idx indexes into query_points.

                Follows the standard GP posterior covariance equation
                (Eq. 3 in Popovic et al. 2020):
                conditioning is on simulated noisy measurements
                z = f + eps, eps ~ N(0, sigma_n_sq), so the block being
                inverted is (c_xx + sigma_n_sq * I), not c_xx alone.
                """
                if not idx:
                    return 0.0
                c_gx = c_g_all[:, idx]  # (M, |idx|)
                c_xx = c_xx_all[np.ix_(idx, idx)]  # (|idx|, |idx|)
                c_xx_noisy = c_xx + sigma_n_sq * np.eye(len(idx))
                y = np.linalg.solve(c_xx_noisy, c_gx.T)  # (|idx|, M)
                return float(np.sum(c_gx * y.T))

            # Increment = additional grid-trace reduction gained by adding
            # the candidate's own new point (last in idx) on top of its
            # ancestors (idx[:-1]). Always >= 0 by construction (Schur
            # complement of c_xx + sigma_n_sq * I, still PSD: conditioning
            # never increases variance), so it slots into node.info =
            # parent.info + increment without changing that accumulation
            # logic. Unlike entropy/log-det, A-optimal trace reduction is
            # not a strictly submodular set function, but for smooth
            # spatial kernels it exhibits the same diminishing-returns
            # behaviour in practice.
            info_increments = []
            for idx in candidate_indices:
                reduction_with_new_point = trace_reduction(idx)
                reduction_ancestors_only = trace_reduction(idx[:-1])
                increment = reduction_with_new_point - reduction_ancestors_only
                info_increments.append(max(increment, 0.0))
            self._log_info_increments(np.array(info_increments))
            return info_increments

        return batch_info_fn

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
            if self.get_parameter('planner_type').value == 'lawnmower':
                self.get_logger().info('Lawnmower coverage finished.')
                self.do_stop()
            else:
                self._end_segment_and_replan('Path end reached')
            return

        if self.get_parameter('planner_type').value == 'lawnmower':
            return

        elapsed = self.get_clock().now(
        ).nanoseconds * 1e-9 - self._last_replan_time
        if elapsed > self.get_parameter('replan_timeout_s').value:
            timeout = self.get_parameter('replan_timeout_s').value
            self._end_segment_and_replan(f'Replan timeout ({timeout} s)')

    def _end_segment_and_replan(self, reason: str):
        self.get_logger().info(f'{reason}, replanning.')
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
        """Sends the planned path to path_follower.

        RIG-tree paths are truncated to the execution horizon (receding-horizon
        replanning). The lawnmower path is a one-time full-coverage path, so
        it is sent in its entirety instead.
        """
        if not path:
            self.get_logger().warn('Empty path, retrying.')
            self.state = State.PLANNING
            return

        if self.get_parameter('planner_type').value == 'lawnmower':
            executed_path = path
        else:
            executed_path = self._truncate_path_to_distance(
                path,
                self.get_parameter('planning_horizon').value)

        ros_path = self.rig_path_to_ros(executed_path)
        req = SetPath.Request()
        req.path = ros_path
        self.set_path_client.call_async(req)

        self._path_end = executed_path[-1]
        self._last_replan_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = State.EXECUTING

        self.publish_path_marker(path)
        self.get_logger().info(
            f'Executing {len(executed_path)} of {len(path)} waypoints.')

    def rig_path_to_ros(self, path: list) -> Path:
        """Converts a list of [x, y] points into a nav_msgs/Path message."""
        z = self.get_parameter('robot_z').value

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        points_3d = [Point(x=float(xy[0]), y=float(xy[1]), z=z) for xy in path]

        orientations = [Quaternion() for _ in points_3d]
        if len(path) >= 2:
            last_yaw = math.atan2(path[-1][1] - path[-2][1],
                                  path[-1][0] - path[-2][0])
            q = quaternion_from_euler(0.0, 0.0, last_yaw)
            orientations[-1] = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

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
