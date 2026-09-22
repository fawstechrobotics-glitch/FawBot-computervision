"""Parsing helpers for LiDAR packets received over robot UDP."""
import json
import math
from typing import Any, Iterable, List, Optional, Tuple


LidarPoint = Tuple[float, float]


def decode_lidar_packet(packet_hex: str) -> Optional[LidarPoint]:
    """Decode the firmware's 5-byte Q6-angle/Q2-distance LiDAR packet."""
    try:
        packet = bytes.fromhex(packet_hex.strip())
    except ValueError:
        return None
    if len(packet) != 5 or packet[0] != ((15 << 2) | 0x01):
        return None

    angle_q6 = ((packet[1] & 0xFE) >> 1) | (packet[2] << 7)
    distance_q2 = packet[3] | (packet[4] << 8)
    return angle_q6 / 64.0, distance_q2 / 40.0


def _point_from_item(item: Any) -> Optional[LidarPoint]:
    if isinstance(item, dict):
        angle = item.get("angle", item.get("angle_deg"))
        distance = item.get("distance", item.get("distance_cm", item.get("range")))
    elif isinstance(item, (list, tuple)) and len(item) >= 2:
        angle, distance = item[0], item[1]
    else:
        return None

    try:
        angle = float(angle)
        distance = float(distance)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(angle) or not math.isfinite(distance) or distance <= 0:
        return None
    return angle, distance


def parse_lidar_message(message: str) -> List[LidarPoint]:
    """Parse supported UDP LiDAR messages into (angle_degrees, distance_cm)."""
    raw = message.strip()
    if not raw:
        return []

    if raw.startswith("{") or raw.startswith("["):
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return []
        if isinstance(payload, dict):
            payload = payload.get("scan", payload.get("points", payload.get("data", [])))
        if isinstance(payload, list):
            return [point for item in payload if (point := _point_from_item(item)) is not None]
        return []

    prefix, separator, payload = raw.partition(":")
    if not separator or prefix.upper() not in {"LIDAR", "LIDAR_SCAN", "SCAN"}:
        return []

    points: Iterable[str]
    if prefix.upper() == "LIDAR":
        points = [payload]
    else:
        points = payload.replace("|", ";").split(";")

    parsed: List[LidarPoint] = []
    for item in points:
        fields = item.strip().replace(",", " ").split()
        if len(fields) >= 2:
            point = _point_from_item(fields[:2])
            if point is not None:
                parsed.append(point)
    return parsed