"""Simple boustrophedon (lawnmower) coverage path, used as a non-informative baseline for RIG-tree."""

from typing import List

import numpy as np


def generate_lawnmower_path(
    x_start: np.ndarray,
    x_max: float,
    y_max: float,
    lane_spacing: float,
    waypoint_spacing: float,
) -> List[np.ndarray]:
    """Builds a coverage path over a rectangular area (no obstacles).

    Lanes run along y (the tank's long axis) and step across in x.
    """
    margin_x = float(x_start[0])
    margin_y = float(x_start[1])

    x_lo, x_hi = margin_x, max(margin_x, x_max - margin_x)
    y_lo, y_hi = margin_y, max(margin_y, y_max - margin_y)

    num_lanes = max(1, round((x_hi - x_lo) / lane_spacing) + 1)
    x_lanes = np.linspace(x_lo, x_hi, num_lanes)

    num_wp = max(2, round((y_hi - y_lo) / waypoint_spacing) + 1)
    y_forward = np.linspace(y_lo, y_hi, num_wp)
    y_backward = y_forward[::-1]

    path: List[np.ndarray] = []
    for i, x in enumerate(x_lanes):
        ys = y_forward if i % 2 == 0 else y_backward
        for y in ys:
            path.append(np.array([x, y]))

    # Snap the first point exactly to the start position.
    path[0] = x_start.copy()

    return path
