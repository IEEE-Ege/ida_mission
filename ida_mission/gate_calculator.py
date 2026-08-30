#!/usr/bin/env python3
# =============================================================================
# gate_calculator.py — Kapı Hesaplama Yardımcısı
# =============================================================================
# Turuncu şamandıra çiftlerinden geçiş noktası (midpoint) ve
# dik başlık (perpendicular heading) quaternion hesaplar.
# =============================================================================

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Pose, Quaternion
from ida_msgs.msg import BuoyDetection


# Varsayılan eşikler — mission_node ROS parametreleri ile geçersiz kılabilir.
DEFAULT_MIN_GATE_WIDTH = 2.0    # metre
DEFAULT_MAX_GATE_WIDTH = 12.0   # metre


def find_gate_pairs(
    buoys: List[BuoyDetection],
    color_filter: str = 'orange',
    min_width: float = DEFAULT_MIN_GATE_WIDTH,
    max_width: float = DEFAULT_MAX_GATE_WIDTH,
) -> List[Tuple[BuoyDetection, BuoyDetection, float]]:
    """
    Renk filtreli şamandıralar arasından geçerli kapı çiftlerini döner.
    Her eleman: (det_a, det_b, mesafe_metre)
    En kısa mesafeli çift önce gelir.
    """
    candidates = [b for b in buoys if b.color == color_filter]
    pairs: List[Tuple[BuoyDetection, BuoyDetection, float]] = []

    for i in range(len(candidates)):
        for j in range(i + 1, len(candidates)):
            a = candidates[i]
            b = candidates[j]
            dist = math.sqrt(
                (a.position.x - b.position.x) ** 2 +
                (a.position.y - b.position.y) ** 2
            )
            if min_width <= dist <= max_width:
                pairs.append((a, b, dist))

    pairs.sort(key=lambda t: t[2])
    return pairs


def gate_midpoint_pose(
    det_a: BuoyDetection,
    det_b: BuoyDetection,
    robot_pos: Optional[Tuple[float, float]] = None,
) -> Pose:
    """
    İki şamandıra arasındaki orta noktayı ve kapıya dik başlığı hesapla.
    Robot konumu verilirse geçiş yönü buna göre seçilir.
    """
    ax, ay = det_a.position.x, det_a.position.y
    bx, by = det_b.position.x, det_b.position.y

    mx = (ax + bx) / 2.0
    my = (ay + by) / 2.0

    gate_dx = bx - ax
    gate_dy = by - ay
    perp_dx = -gate_dy
    perp_dy = gate_dx

    if robot_pos is not None:
        rx, ry = robot_pos
        to_gate_x = mx - rx
        to_gate_y = my - ry
        if perp_dx * to_gate_x + perp_dy * to_gate_y < 0:
            perp_dx = -perp_dx
            perp_dy = -perp_dy

    yaw = math.atan2(perp_dy, perp_dx)
    q = _yaw_to_quaternion(yaw)

    pose = Pose()
    pose.position.x = mx
    pose.position.y = my
    pose.position.z = 0.0
    pose.orientation = q
    return pose


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Yaw (rad) → geometry_msgs/Quaternion (Z ekseninde döndürme)."""
    half = yaw / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def nearest_buoy_by_color(
    buoys: List[BuoyDetection],
    color: str,
    robot_pos: Tuple[float, float],
) -> Optional[BuoyDetection]:
    """Verilen renkteki en yakın şamandırayı döner."""
    candidates = [b for b in buoys if b.color == color]
    if not candidates:
        return None
    rx, ry = robot_pos
    return min(
        candidates,
        key=lambda b: math.hypot(b.position.x - rx, b.position.y - ry),
    )
