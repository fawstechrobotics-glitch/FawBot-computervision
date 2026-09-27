"""2D Computational Geometry, Footprint Calculation, and Collision Validation."""
import math
from typing import List, Tuple, Optional

from models.pose import Pose
from models.obstacle import Boundary, RestrictedArea


def normalize_angle(angle_rad: float) -> float:
    """Normalizes an angle in radians to the range [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalizes an angle in degrees to the range [-180, 180]."""
    rad = math.radians(angle_deg)
    return math.degrees(math.atan2(math.sin(rad), math.cos(rad)))


def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculates Euclidean distance between two 2D points (x1, y1) and (x2, y2)."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def calculate_heading(current_pose: Tuple[float, float], target_point: Tuple[float, float]) -> float:
    """
    Calculates absolute heading angle (in degrees) from current_pose (x, y) 
    to target_point (x, y) in standard Cartesian space (+X East/Right, +Y North/Up).
    """
    dx = target_point[0] - current_pose[0]
    dy = target_point[1] - current_pose[1]  # Standard Cartesian: dy = target_y - current_y
    return math.degrees(math.atan2(dy, dx))


def calculate_turn_angle(current_heading_deg: float, target_point: Tuple[float, float], current_pose: Tuple[float, float]) -> float:
    """
    Calculates relative turn angle (in degrees) required to rotate from current_heading_deg 
    towards target_point. Result is normalized in [-180, 180].
    """
    target_abs = calculate_heading(current_pose, target_point)
    raw_turn = target_abs - current_heading_deg
    return normalize_angle_deg(raw_turn)


def ui_to_cartesian(ui_x: float, ui_y: float, map_height: float, scale_factor: float = 1.0) -> Tuple[float, float]:
    """
    Converts GUI pixel coordinates (Top-Left Origin, +Y Down) 
    to Robot Cartesian coordinates (Bottom-Left Origin, +Y Up).
    """
    cartesian_x = ui_x * scale_factor
    cartesian_y = (map_height - ui_y) * scale_factor
    return cartesian_x, cartesian_y


def compute_robot_footprint(
    x: float,
    y: float,
    heading_deg: float,
    length_cm: float,
    width_cm: float,
    margin_cm: float = 0.0
) -> List[Tuple[float, float]]:
    """
    Calculate the 4 vertices of the oriented robot bounding box,
    including the optional safety margin.
    """
    eff_l = (length_cm + 2.0 * margin_cm) / 2.0
    eff_w = (width_cm + 2.0 * margin_cm) / 2.0

    rad = math.radians(heading_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)

    # Local corners: Front-Left, Back-Left, Back-Right, Front-Right
    local_corners = [
        (eff_l, eff_w),
        (-eff_l, eff_w),
        (-eff_l, -eff_w),
        (eff_l, -eff_w)
    ]

    footprint = []
    for lx, ly in local_corners:
        gx = x + (lx * cos_a - ly * sin_a)
        gy = y + (lx * sin_a + ly * cos_a)
        footprint.append((gx, gy))

    return footprint


def point_in_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    """Ray-casting algorithm to determine whether point is inside or on the boundary of polygon."""
    x, y = point
    n = len(polygon)
    if n < 3:
        return False

    # Check if point lies on any boundary segment
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]
        cross = (x - p1[0]) * (p2[1] - p1[1]) - (y - p1[1]) * (p2[0] - p1[0])
        if abs(cross) < 1e-7:
            if min(p1[0], p2[0]) - 1e-7 <= x <= max(p1[0], p2[0]) + 1e-7 and \
               min(p1[1], p2[1]) - 1e-7 <= y <= max(p1[1], p2[1]) + 1e-7:
                return True

    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if min(p1y, p2y) < y <= max(p1y, p2y):
            if x <= max(p1x, p2x):
                if p1y != p2y:
                    x_inters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if x < x_inters:
                        inside = not inside

        p1x, p1y = p2x, p2y

    return inside


def _ccw(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    """Counter-clockwise orientation test (cross product)."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float]
) -> bool:
    """Test if line segment (p1-p2) intersects line segment (p3-p4)."""
    if (max(p1[0], p2[0]) < min(p3[0], p4[0]) or
        max(p3[0], p4[0]) < min(p1[0], p2[0]) or
        max(p1[1], p2[1]) < min(p3[1], p4[1]) or
        max(p3[1], p4[1]) < min(p1[1], p2[1])):
        return False

    d1 = _ccw(p3, p4, p1)
    d2 = _ccw(p3, p4, p2)
    d3 = _ccw(p1, p2, p3)
    d4 = _ccw(p1, p2, p4)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True

    def on_segment(p, a, b):
        return min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])

    if abs(d1) < 1e-7 and on_segment(p1, p3, p4): return True
    if abs(d2) < 1e-7 and on_segment(p2, p3, p4): return True
    if abs(d3) < 1e-7 and on_segment(p3, p1, p2): return True
    if abs(d4) < 1e-7 and on_segment(p4, p1, p2): return True

    return False


def polygon_intersects_polygon(
    poly1: List[Tuple[float, float]],
    poly2: List[Tuple[float, float]]
) -> bool:
    """Determine if two 2D polygons intersect or if either contains the other."""
    n1 = len(poly1)
    n2 = len(poly2)

    for i in range(n1):
        e1_p1 = poly1[i]
        e1_p2 = poly1[(i + 1) % n1]
        for j in range(n2):
            e2_p1 = poly2[j]
            e2_p2 = poly2[(j + 1) % n2]
            if segments_intersect(e1_p1, e1_p2, e2_p1, e2_p2):
                return True

    if n1 > 0 and point_in_polygon(poly1[0], poly2):
        return True

    if n2 > 0 and point_in_polygon(poly2[0], poly1):
        return True

    return False


def is_footprint_within_boundary(
    footprint: List[Tuple[float, float]],
    boundary: Boundary
) -> bool:
    """Ensure all vertices of the robot footprint remain inside the boundary polygon."""
    if not boundary.enabled or not boundary.polygon:
        return True

    b_poly = boundary.polygon
    min_x, min_y, max_x, max_y = boundary.bounding_box

    for vx, vy in footprint:
        if vx < min_x or vx > max_x or vy < min_y or vy > max_y:
            return False

    if len(b_poly) == 4 and min_x == 0 and min_y == 0:
        return True

    for pt in footprint:
        if not point_in_polygon(pt, b_poly):
            return False

    return True


def validate_trajectory(
    path_points: List[Tuple[float, float, float]],
    robot_length: float,
    robot_width: float,
    safety_margin: float,
    boundary: Boundary,
    restricted_areas: List[RestrictedArea],
    sample_step_cm: float = 2.0
) -> Tuple[bool, List[str]]:
    """Validates complete robot trajectory against environment boundary and restricted areas."""
    errors = []

    if not path_points:
        return True, []

    start_x, start_y, start_h = path_points[0]
    if boundary.enabled and boundary.polygon:
        fp_start = compute_robot_footprint(start_x, start_y, start_h, robot_length, robot_width, safety_margin)
        if not is_footprint_within_boundary(fp_start, boundary):
            errors.append(f"Start Pose ({start_x:.1f}, {start_y:.1f}) exceeds environment boundary.")
    else:
        fp_start = compute_robot_footprint(start_x, start_y, start_h, robot_length, robot_width, safety_margin)

    for ra in restricted_areas:
        if ra.active and ra.is_valid_polygon:
            if polygon_intersects_polygon(fp_start, ra.polygon):
                errors.append(f"Start Pose ({start_x:.1f}, {start_y:.1f}) collides with Restricted Area '{ra.name}'.")

    for idx in range(len(path_points) - 1):
        p1 = path_points[idx]
        p2 = path_points[idx + 1]

        seg_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(1, int(math.ceil(seg_dist / sample_step_cm)))

        seg_collided_ra = set()
        seg_exceeded_boundary = False

        for s in range(steps + 1):
            t = s / steps
            interp_x = p1[0] + (p2[0] - p1[0]) * t
            interp_y = p1[1] + (p2[1] - p1[1]) * t
            
            d_angle = normalize_angle_deg(p2[2] - p1[2])
            interp_h = p1[2] + d_angle * t

            fp = compute_robot_footprint(interp_x, interp_y, interp_h, robot_length, robot_width, safety_margin)

            if boundary.enabled and boundary.polygon:
                if not is_footprint_within_boundary(fp, boundary) and not seg_exceeded_boundary:
                    seg_exceeded_boundary = True
                    errors.append(f"Path segment {idx + 1} exceeds environment boundary near ({interp_x:.1f}, {interp_y:.1f}).")

            for ra in restricted_areas:
                if ra.active and ra.is_valid_polygon and (ra.id not in seg_collided_ra):
                    if polygon_intersects_polygon(fp, ra.polygon):
                        seg_collided_ra.add(ra.id)
                        errors.append(f"Path segment {idx + 1} collides with Restricted Area '{ra.name}' near ({interp_x:.1f}, {interp_y:.1f}).")

    is_valid = (len(errors) == 0)
    return is_valid, errors