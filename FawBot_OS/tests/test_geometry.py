"""Unit tests for computational geometry, footprint collision, and path validation."""
import unittest
from navigation.geometry import (
    compute_robot_footprint,
    point_in_polygon,
    segments_intersect,
    polygon_intersects_polygon,
    is_footprint_within_boundary,
    validate_trajectory
)
from models.obstacle import Boundary, RestrictedArea


class TestGeometry(unittest.TestCase):

    def test_point_in_polygon(self):
        # Square: (0,0) to (10,10)
        poly = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        self.assertTrue(point_in_polygon((5.0, 5.0), poly))
        self.assertTrue(point_in_polygon((0.0, 5.0), poly))  # On edge
        self.assertFalse(point_in_polygon((15.0, 5.0), poly))
        self.assertFalse(point_in_polygon((-1.0, -1.0), poly))

    def test_segments_intersect(self):
        # Crossing segments
        self.assertTrue(segments_intersect((0, 0), (10, 10), (0, 10), (10, 0)))
        # Parallel segments
        self.assertFalse(segments_intersect((0, 0), (10, 0), (0, 2), (10, 2)))
        # Non-intersecting segments
        self.assertFalse(segments_intersect((0, 0), (2, 2), (5, 5), (7, 7)))

    def test_polygon_intersects_polygon(self):
        # Overlapping squares
        poly1 = [(0, 0), (10, 0), (10, 10), (0, 10)]
        poly2 = [(5, 5), (15, 5), (15, 15), (5, 15)]
        self.assertTrue(polygon_intersects_polygon(poly1, poly2))

        # Disjoint squares
        poly3 = [(20, 20), (30, 20), (30, 30), (20, 30)]
        self.assertFalse(polygon_intersects_polygon(poly1, poly3))

        # Contained square (poly4 inside poly1)
        poly4 = [(2, 2), (4, 2), (4, 4), (2, 4)]
        self.assertTrue(polygon_intersects_polygon(poly1, poly4))

    def test_robot_footprint_calculation(self):
        # Center at (10, 10), heading 0 deg, length 10, width 6, margin 0
        fp = compute_robot_footprint(10.0, 10.0, 0.0, 10.0, 6.0, margin_cm=0.0)
        # Expected: corners at (15, 13), (5, 13), (5, 7), (15, 7)
        self.assertEqual(len(fp), 4)
        xs = [p[0] for p in fp]
        ys = [p[1] for p in fp]
        self.assertAlmostEqual(max(xs), 15.0)
        self.assertAlmostEqual(min(xs), 5.0)
        self.assertAlmostEqual(max(ys), 13.0)
        self.assertAlmostEqual(min(ys), 7.0)

    def test_boundary_containment(self):
        boundary = Boundary(width_cm=100.0, height_cm=100.0)
        # Robot well inside
        fp_inside = compute_robot_footprint(50.0, 50.0, 0.0, 12.0, 9.0, margin_cm=2.0)
        self.assertTrue(is_footprint_within_boundary(fp_inside, boundary))

        # Robot exceeding left edge (x=2, half length + margin = 8 -> xmin = -6)
        fp_outside = compute_robot_footprint(2.0, 50.0, 0.0, 12.0, 9.0, margin_cm=2.0)
        self.assertFalse(is_footprint_within_boundary(fp_outside, boundary))

    def test_trajectory_validation_collision(self):
        boundary = Boundary(width_cm=150.0, height_cm=150.0)
        # Restricted area in middle: (40, 40) to (70, 70)
        obs = RestrictedArea(
            id="box_1",
            name="Center Box",
            polygon=[(40, 40), (70, 40), (70, 70), (40, 70)]
        )

        # Path that drives straight through the obstacle: (20, 55) -> (90, 55)
        path_colliding = [(20.0, 55.0, 0.0), (90.0, 55.0, 0.0)]
        is_valid, errors = validate_trajectory(
            path_points=path_colliding,
            robot_length=12.0,
            robot_width=9.0,
            safety_margin=2.0,
            boundary=boundary,
            restricted_areas=[obs]
        )
        self.assertFalse(is_valid)
        self.assertTrue(any("Center Box" in e for e in errors))

        # Clear path around obstacle: (20, 20) -> (90, 20)
        path_clear = [(20.0, 20.0, 0.0), (90.0, 20.0, 0.0)]
        is_valid_clear, errors_clear = validate_trajectory(
            path_points=path_clear,
            robot_length=12.0,
            robot_width=9.0,
            safety_margin=2.0,
            boundary=boundary,
            restricted_areas=[obs]
        )
        self.assertTrue(is_valid_clear)
        self.assertEqual(len(errors_clear), 0)


if __name__ == "__main__":
    unittest.main()
