# utils/box_angle.py
import numpy as np
import math

def _box_center(pts4):
    pts4 = np.asarray(pts4, dtype=float).reshape(4, 2)
    return pts4[:,0].mean(), pts4[:,1].mean()

def _point_to_segment_distance(p, a, b):
    p = np.asarray(p, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    ap = p - a
    ab2 = float(np.dot(ab, ab))
    if ab2 == 0.0:
        return float(np.linalg.norm(ap))
    t = np.clip(np.dot(ap, ab) / ab2, 0.0, 1.0)
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))

def _closest_edge_index_to_point(pts4, target_xy):
    pts4 = np.asarray(pts4, dtype=float).reshape(4, 2)
    edges = [(0,1), (1,2), (2,3), (3,0)]
    dists = [_point_to_segment_distance(target_xy, pts4[i], pts4[j]) for i, j in edges]
    return int(np.argmin(dists))  # 가장 가까운 변 인덱스

def _rotate_points_around_center(pts4, center_xy, delta_rad):
    cx, cy = center_xy
    c, s = math.cos(delta_rad), math.sin(delta_rad)
    out = []
    for (x, y) in np.asarray(pts4, dtype=float).reshape(4, 2):
        dx, dy = x - cx, y - cy
        x2 = cx + c*dx - s*dy
        y2 = cy + s*dx + c*dy
        out.append([x2, y2])
    return np.array(out, dtype=np.float32)

def rotate_box_edge_towards_center_by_midpoint(cls0_xyxyxyxy, cls1_xyxyxyxy, ref_midpoint_xy):
    """
    ref_midpoint_xy: cls1의 긴 변 중점들 중 하나 (edge 선택용)
    동작:
      1) ref_midpoint_xy에 가장 가까운 cls0의 '변'을 고른다.
      2) cls0의 중심(C0) -> cls1의 중심(C1) 방향을 목표 벡터로 잡는다.
      3) 선택된 변의 '법선'이 이 목표 벡터를 향하도록 cls0 전체를 회전한다.
    반환: (rotated_pts4(4,2), new_angle_deg)
    """
    cls0 = np.asarray(cls0_xyxyxyxy, dtype=float).reshape(4, 2)
    cls1 = np.asarray(cls1_xyxyxyxy, dtype=float).reshape(4, 2)

    # 목표 방향: C0 -> C1
    c0 = _box_center(cls0)
    c1 = _box_center(cls1)
    tx, ty = (c1[0] - c0[0], c1[1] - c0[1])
    tgt_theta = math.atan2(ty, tx)

    # ref_midpoint에 가장 가까운 변 선택
    eidx = _closest_edge_index_to_point(cls0, ref_midpoint_xy)
    i, j = eidx, (eidx + 1) % 4

    # 선택된 변의 단위 방향벡터
    vx, vy = (cls0[j] - cls0[i])
    norm = math.hypot(vx, vy) + 1e-9
    ux, uy = vx / norm, vy / norm

    # 두 개의 법선 후보(변의 좌/우)
    normals = [(-uy, ux), (uy, -ux)]

    # 목표 단위 벡터(C0->C1)
    dnorm = math.hypot(tx, ty) + 1e-9
    dx, dy = tx / dnorm, ty / dnorm

    # 목표와 가장 일치(내적 최대)하는 법선 선택
    dots = [nx*dx + ny*dy for (nx, ny) in normals]
    nx, ny = normals[int(np.argmax(dots))]
    curr_theta = math.atan2(ny, nx)  # 현재 "보는" 방향(법선)

    # 필요한 회전량
    delta = tgt_theta - curr_theta

    rotated = _rotate_points_around_center(cls0, c0, delta)
    return rotated, math.degrees(curr_theta + delta)
