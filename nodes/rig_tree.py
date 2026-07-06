"""RIG-tree, Hollinger & Sukhatme, IJRR 2014, Algorithm 2."""

import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class Node:
    x: np.ndarray
    cost: float
    info: float
    parent: Optional[int] = None
    # All positions from the root to this node, in order, including this
    # node's own position as the last entry.
    path_points: List[np.ndarray] = field(
        default_factory=list)  # Fresh list per Node instance


class RIGTree:
    """
    Inputs:
        delta            - step size Delta
        planning_horizon - B (maximum path length / planning horizon)
        x_start          - start position as numpy [x, y] in metres
        R                - near radius
        grid             - 2D numpy array (rows=y, cols=x); cell >= 50 means occupied
        resolution       - metres per cell
        batch_info_fn    - takes one path per candidate (ancestor positions
                           followed by the candidate's own new position) and
                           returns a path-conditioned entropy increment for
                           each. Falls back to uniform +1 per step if None.
    """

    def __init__(
        self,
        delta: float,
        planning_horizon: float,
        x_start: np.ndarray,
        R: float,
        grid: np.ndarray,
        resolution: float,
        batch_info_fn=None,
    ):
        self.delta = delta
        self.planning_horizon = planning_horizon
        self.R = R
        self.R_sq = R * R
        self.eps_sq = (delta / 2.0)**2
        self.grid = grid
        self.resolution = resolution
        self._batch_info_fn = batch_info_fn

        self.x_start = x_start.copy()
        self.height, self.width = grid.shape
        self.x_max = self.width * resolution
        self.y_max = self.height * resolution

        i_init = self._initial_information(x_start)
        # path_points starts empty: the root is not yet part of any scored
        # path
        root = Node(x=x_start.copy(),
                    cost=0.0,
                    info=i_init,
                    parent=None,
                    path_points=[])
        self.nodes: List[Node] = [root]
        self.closed: set = set(
        )  # V_closed: nodes whose path cost exceeds the planning horizon
        self.edges: List[Tuple[int, int]] = []

    # ------------------------------------------------------------------
    # Paper primitives
    # ------------------------------------------------------------------

    def _sample(self) -> np.ndarray:
        """Sample(): uniform in the disk of radius B around x_start,
        rejecting draws that land outside the tank."""
        for _ in range(100):  #100 trys until it lands in tank
            r = self.planning_horizon * math.sqrt(random.random())
            theta = random.uniform(0.0, 2 * math.pi)
            x = self.x_start[0] + r * math.cos(theta)
            y = self.x_start[1] + r * math.sin(theta)
            if 0.0 <= x <= self.x_max and 0.0 <= y <= self.y_max:
                return np.array([x, y])
        return self.x_start.copy()  # fallback, should not normally happen

    def _steer(self, x_from: np.ndarray, x_to: np.ndarray) -> np.ndarray:
        """Steer(x_from, x_to, Delta): move from x_from toward x_to by at most Delta."""
        dx = x_to[0] - x_from[0]
        dy = x_to[1] - x_from[1]
        dist = math.hypot(dx, dy)
        if dist <= self.delta:
            return x_to.copy()
        scale = self.delta / dist
        return np.array([x_from[0] + dx * scale, x_from[1] + dy * scale])

    def _no_collision(self, x_from: np.ndarray, x_to: np.ndarray) -> bool:
        """NoCollision: True if the straight segment lies entirely in free space."""
        dist = math.hypot(x_to[0] - x_from[0], x_to[1] - x_from[1])
        if dist == 0.0:
            return self._cell_free(x_from)
        steps = max(int(dist / (self.resolution * 0.5)), 1)
        for i in range(steps + 1):
            t = i / steps
            if not self._cell_free(x_from + t * (x_to - x_from)):
                return False
        return True

    def _cell_free(self, x: np.ndarray) -> bool:
        """Returns True if the grid cell at position x is free (occupancy value below 50)."""
        col = round(x[0] / self.resolution)
        row = round(x[1] / self.resolution)
        if col < 0 or col >= self.width or row < 0 or row >= self.height:
            return False
        return self.grid[row, col] < 50

    def _initial_information(self, _x_start: np.ndarray) -> float:
        """InitialInformation(x_start, E): information at the root is always zero."""
        return 0.0

    def _evaluate_cost(self, x_from: np.ndarray, x_to: np.ndarray) -> float:
        """EvaluateCost(x_near, x_new): Euclidean distance."""
        return math.hypot(x_to[0] - x_from[0], x_to[1] - x_from[1])

    def _prune(self, n_new: Node, kd: cKDTree, open_arr: np.ndarray) -> bool:
        """PRUNE, modular pruning strategy (Section 4.4).

        Discards n_new if an open node at the same location already exists with
        higher information and equal or lower cost. Co-located threshold is Delta/2.
        KD-tree radius query gives O(log n).
        """
        local_indices = kd.query_ball_point(n_new.x, r=self.delta / 2.0)
        for li in local_indices:
            n_a = self.nodes[open_arr[li]]
            if n_a.info > n_new.info and n_a.cost <= n_new.cost:
                return True
        return False

    # ------------------------------------------------------------------
    # Algorithm 2 main loop
    # ------------------------------------------------------------------

    def run(self,
            planning_time_s: Optional[float] = 1.0,
            cancel_event: Optional[threading.Event] = None) -> dict:
        """Anytime algorithm: runs until stopped ("while not terminated" in the paper).

        Stops when cancel_event is set, or when planning_time_s elapses.
        Pass planning_time_s=None to run with no time limit at all.

        Returns a stats dict: iterations, elapsed_s, saturated (True = tree exhausted before deadline).
        """
        deadline = (time.monotonic() +
                    planning_time_s if planning_time_s is not None else
                    float('inf'))
        t_start = time.monotonic()
        iterations = 0
        nodes_added = 0
        gp_calls = 0
        while (cancel_event is None
               or not cancel_event.is_set()) and time.monotonic() < deadline:
            iterations += 1
            nodes_before = len(self.nodes)
            # Open nodes are those not yet closed by the planning horizon
            open_arr = np.array(
                [i for i in range(len(self.nodes)) if i not in self.closed],
                dtype=int)

            # Build KD-tree once per iteration over open node positions: O(n log n).
            # All spatial queries within this iteration are then O(log n).
            positions = np.array([self.nodes[i].x for i in open_arr])  # (m, 2)
            kd = cKDTree(positions)

            # Lines 7-8: sample, find nearest, steer to feasible point
            x_samp = self._sample()
            _, li_nearest = kd.query(x_samp, k=1)
            nearest_idx = int(open_arr[li_nearest])
            x_feasible = self._steer(self.nodes[nearest_idx].x, x_samp)

            # Lines 10-29: find near nodes and extend each toward x_feasible.
            # Split into three phases so GP queries can be batched (one service
            # call per iteration instead of one per near node).
            near_local = kd.query_ball_point(x_feasible, r=self.R)
            near_arr = open_arr[near_local]

            # Phase 1: geometry, steer and collision check, no GP yet.
            candidates = []
            for near_idx in near_arr:
                n_near = self.nodes[near_idx]
                x_new = self._steer(n_near.x, x_feasible)
                if not self._no_collision(n_near.x, x_new):
                    continue
                c_new = n_near.cost + self._evaluate_cost(n_near.x, x_new)
                path_points = n_near.path_points + [x_new]
                candidates.append((near_idx, x_new, c_new, path_points))

            if not candidates:
                continue

            # Phase 2: GP query, one batch call for all candidates.
            # Falls back to uniform +1 per step when no batch function is available
            # (e.g. GP service not ready).
            if self._batch_info_fn is not None:
                increments = self._batch_info_fn(
                    [p for _, _, _, p in candidates])
                gp_calls += 1
            else:
                increments = [1.0] * len(candidates)

            # Phase 3: build and insert nodes.
            for j, (near_idx, x_new, c_new,
                    path_points) in enumerate(candidates):
                n_near = self.nodes[near_idx]
                i_new = n_near.info + increments[j]

                n_new = Node(x=x_new,
                             cost=c_new,
                             info=i_new,
                             parent=near_idx,
                             path_points=path_points)
                if self._prune(n_new, kd, open_arr):
                    continue

                new_idx = len(self.nodes)
                self.nodes.append(n_new)
                self.edges.append((near_idx, new_idx))

                if c_new > self.planning_horizon:
                    self.closed.add(new_idx)

            nodes_added += len(self.nodes) - nodes_before

        return {
            'iterations': iterations,
            'elapsed_s': time.monotonic() - t_start,
            'nodes_added': nodes_added,
            'nodes': len(self.nodes),
            'closed': len(self.closed),
            'gp_calls': gp_calls,
            'stopped_reason':
            'cancelled' if
            (cancel_event is not None and cancel_event.is_set()) else
            'deadline',
        }

    # ------------------------------------------------------------------
    # Result extraction
    # ------------------------------------------------------------------

    def best_path(self) -> List[np.ndarray]:
        """Reconstructs the most informative completed trajectory."""
        if not self.closed:
            return []

        # key compares mean entropy (info/cost) of closed paths, not the raw index
        best_idx = max(self.closed,
                       key=lambda i: self.nodes[i].info / self.nodes[i].cost)
        idx: Optional[int] = best_idx

        # Reconstruct most informativepath by following parent links back to the root
        path = []
        while idx is not None:
            path.append(self.nodes[idx].x.copy())
            idx = self.nodes[idx].parent
        path.reverse()

        return path
