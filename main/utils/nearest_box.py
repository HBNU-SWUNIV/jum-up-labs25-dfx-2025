import numpy as np

def point_to_segment_distance(p, a, b):
    """
    p, a, b: (2,) numpy array
    return: 점 p와 선분 ab 사이 최소거리
    """
    p = np.asarray(p, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    ap = p - a
    ab2 = np.dot(ab, ab)
    if ab2 == 0.0:  # 퇴화
        return np.linalg.norm(ap)
    t = np.clip(np.dot(ap, ab) / ab2, 0.0, 1.0)
    proj = a + t * ab
    return np.linalg.norm(p - proj)

def dist_point_to_box_boundary(p, box_xyxyxyxy):
    """
    p: (x,y)
    box_xyxyxyxy: [x1,y1,x2,y2,x3,y3,x4,y4] or [[x1,y1],...,[x4,y4]]
    return: 점 p에서 박스 '테두리'까지 최소거리
    """
    pts = np.asarray(box_xyxyxyxy, dtype=float).reshape(4, 2)
    edges = [(0,1), (1,2), (2,3), (3,0)]
    dists = [point_to_segment_distance(p, pts[i], pts[j]) for i, j in edges]
    return min(dists)

def assign_points_to_nearest_box(points, boxes_xyxyxyxy, max_dist=None):
    """
    points: [(x,y), ...]
    boxes_xyxyxyxy: [[x1,y1,...,x4,y4], ...]
    max_dist: 임계값 초과 시 -1 (미할당)
    return: (idxs, dists)
    idxs[i]  = i번째 점에 가장 가까운 박스 인덱스(없으면 -1)
    dists[i] = 그 최소거리
    """
    if len(boxes_xyxyxyxy) == 0:
        return [-1]*len(points), [float('inf')]*len(points)

    idxs, dists = [], []
    for p in points:
        ds = [dist_point_to_box_boundary(p, b) for b in boxes_xyxyxyxy]
        k = int(np.argmin(ds))
        dmin = ds[k]
        if (max_dist is not None) and (dmin > max_dist):
            idxs.append(-1)
            dists.append(dmin)
        else:
            idxs.append(k)
            dists.append(dmin)
    return idxs, dists





#########################################################################

def edge_midpoints(pts4):
    """
    pts4: (4,2) 꼭짓점(시계/반시계 무관)
    return: (4,2) 각 변(0-1,1-2,2-3,3-0)의 중점
    """
    pts = np.asarray(pts4, dtype=float).reshape(4,2)
    mids = []
    for i in range(4):
        j = (i + 1) % 4
        mx = (pts[i,0] + pts[j,0]) / 2.0
        my = (pts[i,1] + pts[j,1]) / 2.0
        mids.append([mx, my])
    return np.array(mids, dtype=float)

def closest_edge_midpoints(boxA_pts4, boxB_pts4):
    """
    두 박스의 변 중점들 사이에서 가장 가까운 쌍을 반환
    return: (midA, midB, idxA, idxB, dist)
    """
    mA = edge_midpoints(boxA_pts4)  # (4,2)
    mB = edge_midpoints(boxB_pts4)  # (4,2)
    # 거리 행렬 (4x4)
    diff = mA[:, None, :] - mB[None, :, :]
    dmat = np.linalg.norm(diff, axis=2)  # (4,4)
    ia, ib = np.unravel_index(np.argmin(dmat), dmat.shape)
    return mA[ia], mB[ib], ia, ib, float(dmat[ia, ib])
