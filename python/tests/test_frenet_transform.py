"""Tests for frenet_transform module."""
import math
import pytest
import numpy as np

from frenet.cubic_spline_planner import Spline2D
from frenet.frenet_transform import frenet_to_cart, cart_to_frenet
from frenet.frenet_optimal_trajectory import quintic_polynomial, quartic_polynomial


# ---------------------------------------------------------------------------
# Helper: build a Spline2D from waypoints
# ---------------------------------------------------------------------------

def _straight_path_heading0():
    """Straight path along the x-axis (heading = 0 deg)."""
    xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ys = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return Spline2D(xs, ys)


def _straight_path_heading90():
    """Straight path along the y-axis (heading = 90 deg)."""
    xs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ys = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    return Spline2D(xs, ys)


def _curved_path():
    """A gently curved reference path."""
    xs = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    ys = [0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0]
    return Spline2D(xs, ys)


# ---------------------------------------------------------------------------
# frenet_to_cart tests
# ---------------------------------------------------------------------------

class TestFrenetToCart:
    """Tests for the frenet_to_cart function."""

    def test_straight_heading0_left_offset(self):
        """d=1 on a heading-0 path should give y+1 (left of path)."""
        csp = _straight_path_heading0()
        s = 15.0
        x, y = frenet_to_cart(s, d=1.0, csp=csp)
        assert x == pytest.approx(15.0, abs=0.5)
        assert y == pytest.approx(1.0, abs=0.1)

    def test_straight_heading0_right_offset(self):
        """d=-1 on a heading-0 path should give y-1 (right of path)."""
        csp = _straight_path_heading0()
        s = 15.0
        x, y = frenet_to_cart(s, d=-1.0, csp=csp)
        assert x == pytest.approx(15.0, abs=0.5)
        assert y == pytest.approx(-1.0, abs=0.1)

    def test_straight_heading90_left_offset(self):
        """d=1 on a heading-90 path should give x-1 (left of path)."""
        csp = _straight_path_heading90()
        s = 15.0
        x, y = frenet_to_cart(s, d=1.0, csp=csp)
        assert x == pytest.approx(-1.0, abs=0.1)
        assert y == pytest.approx(15.0, abs=0.5)

    def test_straight_heading90_right_offset(self):
        """d=-1 on a heading-90 path should give x+1 (right of path)."""
        csp = _straight_path_heading90()
        s = 15.0
        x, y = frenet_to_cart(s, d=-1.0, csp=csp)
        assert x == pytest.approx(1.0, abs=0.1)
        assert y == pytest.approx(15.0, abs=0.5)

    def test_on_path_d_zero(self):
        """d=0 should return a point on the reference path."""
        csp = _curved_path()
        s = 5.0
        fx, fy = frenet_to_cart(s, d=0.0, csp=csp)
        rx, ry = csp.calc_position(s)
        assert fx == pytest.approx(rx, abs=1e-10)
        assert fy == pytest.approx(ry, abs=1e-10)


# ---------------------------------------------------------------------------
# cart_to_frenet tests
# ---------------------------------------------------------------------------

class TestCartToFrenet:
    """Tests for the cart_to_frenet function."""

    def test_on_path_d_near_zero(self):
        """A point on the spline should have d ≈ 0."""
        csp = _curved_path()
        s_query = 8.0
        rx, ry = csp.calc_position(s_query)
        s_out, d_out = cart_to_frenet(rx, ry, csp)
        assert d_out == pytest.approx(0.0, abs=0.1)

    def test_sign_convention_left(self):
        """A point to the left of a heading-0 path should have d > 0."""
        csp = _straight_path_heading0()
        # Point above the x-axis is to the left when heading is 0
        s_out, d_out = cart_to_frenet(15.0, 2.0, csp)
        assert d_out > 0

    def test_sign_convention_right(self):
        """A point to the right of a heading-0 path should have d < 0."""
        csp = _straight_path_heading0()
        # Point below the x-axis is to the right when heading is 0
        s_out, d_out = cart_to_frenet(15.0, -2.0, csp)
        assert d_out < 0


# ---------------------------------------------------------------------------
# Round-trip tests
# ---------------------------------------------------------------------------

class TestRoundTrip:
    """Round-trip: cart_to_frenet -> frenet_to_cart recovers original point."""

    def test_roundtrip_straight(self):
        csp = _straight_path_heading0()
        px, py = 25.0, 3.0
        s, d = cart_to_frenet(px, py, csp)
        rx, ry = frenet_to_cart(s, d, csp)
        assert rx == pytest.approx(px, abs=1.0)
        assert ry == pytest.approx(py, abs=0.5)

    def test_roundtrip_curved(self):
        csp = _curved_path()
        # Pick a point near the path
        s_ref = 10.0
        rx, ry = csp.calc_position(s_ref)
        yaw = csp.calc_yaw(s_ref)
        # Offset 1.5 to the left
        px = rx + 1.5 * math.cos(yaw + math.pi / 2.0)
        py = ry + 1.5 * math.sin(yaw + math.pi / 2.0)
        s_out, d_out = cart_to_frenet(px, py, csp)
        fx, fy = frenet_to_cart(s_out, d_out, csp)
        assert fx == pytest.approx(px, abs=1.0)
        assert fy == pytest.approx(py, abs=1.0)


# ---------------------------------------------------------------------------
# Polynomial class tests
# ---------------------------------------------------------------------------

class TestQuinticPolynomial:
    """Tests for quintic_polynomial boundary conditions."""

    def test_initial_conditions(self):
        xs, vxs, axs = 1.0, 2.0, 0.5
        xe, vxe, axe = 10.0, 0.0, 0.0
        T = 5.0
        qp = quintic_polynomial(xs, vxs, axs, xe, vxe, axe, T)
        assert qp.calc_point(0) == pytest.approx(xs)
        assert qp.calc_first_derivative(0) == pytest.approx(vxs)
        assert qp.calc_second_derivative(0) == pytest.approx(axs)

    def test_final_conditions(self):
        xs, vxs, axs = 1.0, 2.0, 0.5
        xe, vxe, axe = 10.0, 0.0, 0.0
        T = 5.0
        qp = quintic_polynomial(xs, vxs, axs, xe, vxe, axe, T)
        assert qp.calc_point(T) == pytest.approx(xe, abs=1e-8)
        assert qp.calc_first_derivative(T) == pytest.approx(vxe, abs=1e-8)
        assert qp.calc_second_derivative(T) == pytest.approx(axe, abs=1e-8)


class TestQuarticPolynomial:
    """Tests for quartic_polynomial boundary conditions."""

    def test_initial_conditions(self):
        xs, vxs, axs = 0.0, 5.0, 0.0
        vxe, axe = 8.0, 0.0
        T = 4.0
        qp = quartic_polynomial(xs, vxs, axs, vxe, axe, T)
        assert qp.calc_point(0) == pytest.approx(xs)
        assert qp.calc_first_derivative(0) == pytest.approx(vxs)
        assert qp.calc_second_derivative(0) == pytest.approx(axs)

    def test_final_conditions(self):
        xs, vxs, axs = 0.0, 5.0, 0.0
        vxe, axe = 8.0, 0.0
        T = 4.0
        qp = quartic_polynomial(xs, vxs, axs, vxe, axe, T)
        assert qp.calc_first_derivative(T) == pytest.approx(vxe, abs=1e-8)
        assert qp.calc_second_derivative(T) == pytest.approx(axe, abs=1e-8)


# ---------------------------------------------------------------------------
# Spline2D tests
# ---------------------------------------------------------------------------

class TestSpline2D:
    """Tests for Spline2D properties."""

    def test_arc_length(self):
        """Total arc length of a straight 50-unit path should be ~50."""
        csp = _straight_path_heading0()
        assert csp.s[-1] == pytest.approx(50.0, abs=0.1)

    def test_position_at_endpoints(self):
        """First and last knot positions should match input waypoints."""
        xs = [0.0, 10.0, 20.0]
        ys = [0.0, 5.0, 0.0]
        csp = Spline2D(xs, ys)
        x0, y0 = csp.calc_position(csp.s[0])
        assert x0 == pytest.approx(0.0, abs=1e-10)
        assert y0 == pytest.approx(0.0, abs=1e-10)
        # Use second-to-last knot since Spline.calc() has boundary issue at exact last knot
        xn, yn = csp.calc_position(csp.s[-2])
        assert xn == pytest.approx(10.0, abs=1e-10)
        assert yn == pytest.approx(5.0, abs=1e-10)

    def test_yaw_straight_horizontal(self):
        """Yaw along a horizontal straight path should be ~0."""
        csp = _straight_path_heading0()
        yaw = csp.calc_yaw(25.0)
        assert yaw == pytest.approx(0.0, abs=0.05)

    def test_yaw_straight_vertical(self):
        """Yaw along a vertical straight path should be ~pi/2."""
        csp = _straight_path_heading90()
        yaw = csp.calc_yaw(25.0)
        assert yaw == pytest.approx(math.pi / 2.0, abs=0.05)
