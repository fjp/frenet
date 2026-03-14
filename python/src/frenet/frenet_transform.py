"""
Standalone Frenet <-> Cartesian coordinate transform functions.

Uses Spline2D from cubic_spline_planner as the reference path representation.

Sign convention: d > 0 means the point is to the LEFT of the path
(90 deg counter-clockwise from the tangent direction).
"""
import math
import numpy as np


def frenet_to_cart(s, d, csp):
    """Convert Frenet (s, d) to Cartesian (x, y) using a Spline2D reference path.

    Args:
        s: Arc-length coordinate along the reference path.
        d: Signed lateral offset (positive = left of path).
        csp: A Spline2D instance representing the reference path.

    Returns:
        (x, y) Cartesian coordinates.
    """
    ix, iy = csp.calc_position(s)
    yaw = csp.calc_yaw(s)
    fx = ix + d * math.cos(yaw + math.pi / 2.0)
    fy = iy + d * math.sin(yaw + math.pi / 2.0)
    return fx, fy


def cart_to_frenet(px, py, csp, ds=0.1):
    """Convert Cartesian (x, y) to Frenet (s, d) using a Spline2D reference path.

    Samples the spline at fine arc-length intervals, finds the closest sample,
    then computes the signed lateral distance using the left-normal convention.

    Args:
        px: x coordinate of the query point.
        py: y coordinate of the query point.
        csp: A Spline2D instance representing the reference path.
        ds: Sampling interval along the spline arc-length (default 0.1).

    Returns:
        (s, d) Frenet coordinates where s is the arc-length and d is the
        signed lateral offset (positive = left of path).
    """
    # Sample the spline at fine intervals for accurate closest-point search
    s_samples = np.arange(0.0, csp.s[-1], ds)
    min_dist = float("inf")
    s_closest = 0.0

    for s_val in s_samples:
        rx, ry = csp.calc_position(s_val)
        dist = math.hypot(px - rx, py - ry)
        if dist < min_dist:
            min_dist = dist
            s_closest = s_val

    # Compute signed lateral distance
    rx, ry = csp.calc_position(s_closest)
    yaw = csp.calc_yaw(s_closest)

    # Vector from reference point to query point
    dx = px - rx
    dy = py - ry

    # Project onto left-normal direction: n = (-sin(yaw), cos(yaw))
    d_signed = -math.sin(yaw) * dx + math.cos(yaw) * dy

    return s_closest, d_signed
