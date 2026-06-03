"""RIG-tree, Hollinger & Sukhatme, IJRR 2014, Algorithm 2."""

import math
import random
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class Node:
    x: np.ndarray
    cost: float
    info: float
    parent: Optional[int] = None


class RIGTree:
    """
    Inputs:
        delta            - step size Delta
        planning_horizon - B (maximum path length / planning horizon)
        x_start          - start position as numpy [x, y] in metres
        R                - near radius
        grid             - 2D numpy array (rows=y, cols=x); cell >= 50 means occupied
        resolution       - metres per cell
        info_fn          - optional callable(i_near, x_new) -> float; default: +1 per step
    """

    def __init__(
        self,
        delta: float,
        planning_horizon: float,
        x_start: np.ndarray,
        R: float,
        grid: np.ndarray,
        resolution: float,
        info_fn=None,
    ):
        self.delta = delta
        self.planning_horizon = planning_horizon
        self.R = R
        self.R_sq = R * R
        self.eps_sq = (delta / 2.0) ** 2
        self.grid = grid
        self.resolution = resolution
        self._info_fn = info_fn if info_fn is not None else (lambda i_near, _: i_near + 1.0)

        self.height, self.width = grid.shape
        self.x_max = self.width * resolution
        self.y_max = self.height * resolution

        i_init = self._initial_information(x_start)
        root = Node(x=x_start.copy(), cost=0.0, info=i_init, parent=None)
        self.nodes: List[Node] = [root]
        self.closed: set = set()  # V_closed: nodes whose path cost exceeds the planning horizon
        self.edges: List[Tuple[int, int]] = []

    # ------------------------------------------------------------------
    # Paper primitives
    # ------------------------------------------------------------------

    def _sample(self) -> np.ndarray:
        """Sample(X_all): uniform random point in the workspace."""
        return np.array([random.uniform(0.0, self.x_max),
                         random.uniform(0.0, self.y_max)])

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

    def _information(self, i_near: float, x_new: np.ndarray) -> float:
        """Information(I_near, x_new, E): delegates to info_fn."""
        return self._info_fn(i_near, x_new)

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

    def run(self, planning_time_s: float = 1.0):
        """Anytime algorithm: runs until the time limit expires ("while not terminated" in the paper)."""
        deadline = time.monotonic() + planning_time_s
        while time.monotonic() < deadline:
            # Open nodes are those not yet closed by the planning horizon
            open_arr = np.array([i for i in range(len(self.nodes))
                                 if i not in self.closed], dtype=int)
            if len(open_arr) == 0:
                break

            # Build KD-tree once per iteration over open node positions: O(n log n).
            # All spatial queries within this iteration are then O(log n).
            positions = np.array([self.nodes[i].x for i in open_arr])  # (m, 2)
            kd = cKDTree(positions)

            # Lines 7-8: sample, find nearest, steer to feasible point
            x_samp = self._sample()
            _, li_nearest = kd.query(x_samp, k=1)
            nearest_idx = int(open_arr[li_nearest])
            x_feasible = self._steer(self.nodes[nearest_idx].x, x_samp)

            # Lines 10-29: find near nodes and extend each toward x_feasible
            near_local = kd.query_ball_point(x_feasible, r=self.R)
            near_arr = open_arr[near_local]

            for near_idx in near_arr:
                n_near = self.nodes[near_idx]
                x_new = self._steer(n_near.x, x_feasible)

                if not self._no_collision(n_near.x, x_new):
                    continue

                i_new = self._information(n_near.info, x_new)
                c_new = n_near.cost + self._evaluate_cost(n_near.x, x_new)
                n_new = Node(x=x_new, cost=c_new, info=i_new, parent=near_idx)

                if self._prune(n_new, kd, open_arr):
                    continue

                new_idx = len(self.nodes)
                self.nodes.append(n_new)
                self.edges.append((near_idx, new_idx))

                if c_new > self.planning_horizon:
                    self.closed.add(new_idx)

    # ------------------------------------------------------------------
    # Result extraction
    # ------------------------------------------------------------------

    def best_path(self) -> List[np.ndarray]:
        """Reconstructs the most informative completed trajectory."""
        if not self.closed:
            return []
        best_idx = max(self.closed, key=lambda i: self.nodes[i].info)
        path = []
        idx: Optional[int] = best_idx
        while idx is not None:
            path.append(self.nodes[idx].x.copy())
            idx = self.nodes[idx].parent
        path.reverse()
        return path
