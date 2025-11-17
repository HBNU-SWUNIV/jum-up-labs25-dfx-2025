import pyrealsense2 as rs
import cv2
import numpy as np
import math

from indy7 import indyCTL

from ultralytics import YOLO

from utils.box_angle import rotate_box_edge_towards_center_by_midpoint
from utils.nearest_box import assign_points_to_nearest_box, closest_edge_midpoints

class camera():
    def __init__(self):
        super().__init__()
        # RealSense 파이프라인 초기화
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,    30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,     30)
        self.align = rs.align(rs.stream.color)
        self.click_point = None
        self.use_filters = False
        if self.use_filters:
            # self.decimate  = rs.decimation_filter()   # 다운샘플로 노이즈 완화
            self.spatial   = rs.spatial_filter()      # 공간 필터
            self.temporal  = rs.temporal_filter()     # 시간 필터
            self.holefill  = rs.hole_filling_filter() # 홀 채움

        self.indy = indyCTL()
        self.model = YOLO("model/best.pt")

    def draw_obb_point_list(self, image, points, color=(255, 0, 0), thickness=1):
        """
        points: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]] 형식
        """
        pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))  # OpenCV 포맷
        cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness)
        return image
    
    def center_two_point_draw(self, image, points):
        output_points = []
        pts = np.array(points, dtype=np.float32).reshape(4, 2)

        # ───── 가장 긴 변 두 개의 중점 계산 ─────
        edges = [(i, (i + 1) % 4) for i in range(4)]
        lengths = [np.linalg.norm(pts[j] - pts[i]) for i, j in edges]
        max_len = max(lengths)

        # 가장 긴 두 변 찾기
        idxs = [k for k, l in enumerate(lengths) if abs(l - max_len) < 1e-3 * max_len]
        if len(idxs) == 1:  # 혹시 한 개만 잡히면 반대편 변 추가
            idxs.append((idxs[0] + 2) % 4)
        elif len(idxs) > 2:  # 정사각형 등일 경우 첫 변과 반대 변만
            idxs = [idxs[0], (idxs[0] + 2) % 4]

        # 중점 계산 및 표시
        for k in idxs:
            i, j = edges[k]
            mx = (pts[i, 0] + pts[j, 0]) / 2.0
            my = (pts[i, 1] + pts[j, 1]) / 2.0
            cv2.circle(image, (int(mx), int(my)), 4, (0, 0, 255), -1)
            output_points.append([int(mx), int(my)])

        return output_points

    def fold_angle(self, angle_deg):
        """
        입력: -360° ~ +360°
        출력: -90° ~ +90°
        예: 135° → -45°, 270° → 90°, -135° → 45°
        """
        # 1. -360~360 → 0~360으로 정규화
        angle = angle_deg % 360

        # 2. 180도 초과는 반대 방향으로 접기
        if angle > 180:
            angle -= 360  # 270° → -90°, 315° → -45°

        # 3. 이제 -180~180 사이
        # 90° 초과 혹은 -90° 미만인 경우, 대칭 변환
        if angle > 90:
            angle = 180 - angle
        elif angle < -90:
            angle = -180 - angle

        return angle

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:   # 왼쪽 클릭 시
            print(f"Clicked pixel: (u={x}, v={y})")
            self.click_point = True

    def angles_from_pixel(self, depth_frame, u=200, v=200):
        w = depth_frame.get_width()
        h = depth_frame.get_height()
        u = int(max(0, min(w - 1, round(u))))
        v = int(max(0, min(h - 1, round(v))))
        z = depth_frame.get_distance(u, v)
        if z <= 0:
            return None

        # 반드시 "depth 프레임의" intrinsics 사용
        depth_vsp  = depth_frame.get_profile().as_video_stream_profile()
        depth_intr = depth_vsp.get_intrinsics()
        X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intr, [u, v], z)
        print(depth_intr)

        theta = math.degrees(math.atan2(math.hypot(X, Y), Z))
        yaw   = math.degrees(math.atan2(X, Z))
        pitch = math.degrees(math.atan2(-Y, math.hypot(X, Z)))
        return dict(distance_m=z, X=X, Y=Y, Z=Z,
                    theta_deg=theta, yaw_deg=yaw, pitch_deg=pitch)

    def start(self):
        self.profile = self.pipeline.start(self.config)
        self.device = self.profile.get_device()

        # 제품군 확인: "L500" 또는 "D400"
        product_line = self.device.get_info(rs.camera_info.product_line)

        # 공통: depth sensor 핸들
        self.depth_sensor = self.device.first_depth_sensor()

        # 제품군별 설정
        if product_line == "L500":
            # L515 전용 프리셋(사용 중 코드 유지)
            try:
                self.depth_sensor.set_option(rs.option.visual_preset,
                                             rs.l500_visual_preset.short_range)
            except Exception:
                pass
        else:
            # D415(D400 시리즈): L500 프리셋 없음 → 에미터/레이저 파워만 적절히 설정
            try:
                # 에미터 켜기(깊이 품질 향상, 필요 시 0으로 꺼도 됨)
                if self.depth_sensor.supports(rs.option.emitter_enabled):
                    self.depth_sensor.set_option(rs.option.emitter_enabled, 1)
                # 레이저 파워(장비/환경 따라 조절, 1~ 최대치 범위. 과포화시 낮추기)
                if self.depth_sensor.supports(rs.option.laser_power):
                    rng = self.depth_sensor.get_option_range(rs.option.laser_power)
                    self.depth_sensor.set_option(rs.option.laser_power, min(200, rng.max))
            except Exception:
                pass

        cv2.namedWindow("View")
        cv2.setMouseCallback("View", self.mouse_callback)

        colorizer = rs.colorizer()  # 시각화용
        colorizer.set_option(rs.option.color_scheme, 0)

        while True:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            depth_for_calc = depth_frame
            depth_for_view = depth_frame

            if self.use_filters:
                # depth_for_view = self.decimate.process(depth_for_view)
                depth_for_view = self.spatial.process(depth_for_view)
                depth_for_view = self.temporal.process(depth_for_view)
                depth_for_view = self.holefill.process(depth_for_view)

            color_np = np.asanyarray(color_frame.get_data())
            depth_vis = np.asanyarray(colorizer.colorize(depth_for_view).get_data())

            view_color = color_np.copy()

            img = cv2.resize(view_color, (320,320))
            show_img1 = img.copy()
            show_img2 = img.copy()
            show_img3 = img.copy()

            result = self.model.predict(img, conf=0.7)

            output_points   = []
            harvest_leaves  = []
            cls1_boxes      = []

            print(result[0].obb.cls)

            # step 1: Center_leaf 양쪽 변 중앙점 구하기
            for points, cls in zip(result[0].obb.xyxyxyxy.tolist(), result[0].obb.cls.tolist()):
                print(cls, points)
                if cls == 0:
                    self.draw_obb_point_list(image=show_img1, points=points)
                    harvest_leaves.append(points)
                else:
                    self.draw_obb_point_list(image=show_img1, points=points, color=(0,255,0))
                    out = self.center_two_point_draw(image=show_img1, points=points)
                    output_points.append(out)
                    cls1_boxes.append(points)

            # step 2: Center_leaf와 가장 가까운 Harvest_leaf를 한 개체로 만들기
            flat_points = [tuple(pt) for pair in output_points for pt in pair]
            idxs, dists = assign_points_to_nearest_box(flat_points, harvest_leaves, max_dist=None)

            entities = []
            for i, (pts2, cls1_box) in enumerate(zip(output_points, cls1_boxes)):
                pair_idxs  = idxs[2*i:2*i+2]    # [k1, k2]
                pair_dists = dists[2*i:2*i+2]   # [d1, d2]
                pair_boxes = [harvest_leaves[k] if k != -1 else None for k in pair_idxs]

                # ── 엔티티 내부 유니크 강제: 같은 cls0를 두 중점이 가리키면 가까운 쪽만 유지
                if pair_idxs[0] != -1 and pair_idxs[0] == pair_idxs[1]:
                    if pair_dists[0] <= pair_dists[1]:
                        # 0번 포인트 승자, 1번 포인트 탈락
                        pair_idxs[1]  = -1
                        pair_dists[1] = float('inf')
                        pair_boxes[1] = None
                    else:
                        # 1번 포인트 승자, 0번 포인트 탈락
                        pair_idxs[0]  = -1
                        pair_dists[0] = float('inf')
                        pair_boxes[0] = None

                entity = {
                    "cls1_index": i,
                    "cls1_box": cls1_box,
                    "midpoints": pts2,
                    "paired_cls0_idxs": pair_idxs,     # 엔티티 내부 유니크 보장
                    "paired_cls0_boxes": pair_boxes,
                    "paired_dists": pair_dists,
                }
                entities.append(entity)

            # idxs, dists 는 flat_points 기준
            # m-th flat point → 엔티티 i = m//2, mid j = m%2
            candidates = []
            for m, (bi, dm) in enumerate(zip(idxs, dists)):
                if bi == -1:
                    continue
                i, j = divmod(m, 2)  # 엔티티/중점 인덱스
                candidates.append((dm, i, j, bi))  # (거리, cls1_i, mid_j, cls0_idx)

            # 가까운 순 정렬 후, 승자만 남김
            candidates.sort(key=lambda x: x[0])

            used_cls0 = set()
            assigned = {}  # (i,j) -> cls0_idx
            for dm, i, j, bi in candidates:
                if bi in used_cls0:
                    continue
                if (i, j) in assigned:  # 안전망 (중복 방지)
                    continue
                used_cls0.add(bi)
                assigned[(i, j)] = bi

            # 결과를 entities에 반영 (미승자는 -1 처리)
            for i, e in enumerate(entities):
                for j in range(2):
                    if (i, j) in assigned:
                        bi = assigned[(i, j)]
                        e["paired_cls0_idxs"][j] = bi
                        e["paired_cls0_boxes"][j] = harvest_leaves[bi]
                        # dists 업데이트(선택): 이미 dm 알면 넣고, 아니면 그대로
                    else:
                        e["paired_cls0_idxs"][j] = -1
                        e["paired_cls0_boxes"][j] = None
                        e["paired_dists"][j] = float('inf')


            for (x, y), bi in zip(flat_points, idxs):
                cv2.circle(show_img1, (int(x), int(y)), 4, (0, 255, 255), -1)  # 점 강조
                if bi != -1:
                    # 선택된 박스의 테두리까지 최단점(투영점)을 구해 선으로 그려도 좋지만,
                    # 간단히 박스 중심으로 보조선만 표시:
                    box = np.array(harvest_leaves[bi], dtype=float).reshape(4,2)
                    bx, by = box[:,0].mean(), box[:,1].mean()
                    cv2.line(show_img1, (int(x), int(y)), (int(bx), int(by)), (255, 255, 0), 1)

            # 디버그 출력
            print(f"#entities = {len(entities)}")
            for e in entities:
                print({
                    "cls1_index": e["cls1_index"],
                    "paired_cls0_idxs": e["paired_cls0_idxs"],
                    "paired_dists": [round(d,2) for d in e["paired_dists"]],
                })

            # Step 3: Harvest_leaf가 Center_leaf의 중심을 바라보게 하기
            for e in entities:
                mids = e["midpoints"]              # [[mx1,my1],[mx2,my2]]  ← cls1 긴 변 중점 2개
                cls1_box = e["cls1_box"]

                e["rotated_cls0_boxes"] = []
                e["rotated_angles"] = []

                for j, cls0_box in enumerate(e["paired_cls0_boxes"]):
                    if cls0_box is None:
                        e["rotated_cls0_boxes"].append(None)
                        e["rotated_angles"].append(None)
                        continue

                    ref_mid = tuple(mids[j])  # 이 중점에 가장 가까운 변을 선택
                    rotated_cls0, theta_deg = rotate_box_edge_towards_center_by_midpoint(
                        cls0_box, cls1_box, ref_midpoint_xy=ref_mid
                    )

                    e["rotated_cls0_boxes"].append(rotated_cls0)
                    e["rotated_angles"].append(theta_deg)

                    # 시각화(선택)
                    cv2.polylines(show_img2, [rotated_cls0.astype(np.int32).reshape(-1,1,2)], True, (0,128,255), 2)
                    # 보조선: 회전된 cls0 중심 → cls1 중심
                    c0 = rotated_cls0.mean(axis=0)
                    c1 = np.array(cls1_box, float).reshape(4,2).mean(axis=0)
                    cv2.circle(show_img2, (int(c0[0]), int(c0[1])), 3, (255,255,0), -1)
                    cv2.circle(show_img2, (int(c1[0]), int(c1[1])), 3, (0,255,255), -1)
                    cv2.line(show_img2, (int(c0[0]), int(c0[1])), (int(c1[0]), int(c1[1])), (255,255,0), 1)

            # Step 4: Harvest_leaf를 서로 연결하고 중심을 구해 깻잎개체의 중심 구하기
            line_info_list = []  # [( (cx, cy), slope ), ...]

            for e in entities:
                rb0, rb1 = None, None
                if len(e.get("rotated_cls0_boxes", [])) >= 2:
                    rb0, rb1 = e["rotated_cls0_boxes"][0], e["rotated_cls0_boxes"][1]

                # 두 박스가 모두 있어야 연결 가능
                if rb0 is None or rb1 is None:
                    continue

                # 두 박스에서 가장 가까운 변의 중점 계산
                midA, midB, ia, ib, dmin = closest_edge_midpoints(rb0, rb1)

                # 중점 좌표 (정수화)
                pA = (int(round(midA[0])), int(round(midA[1])))
                pB = (int(round(midB[0])), int(round(midB[1])))

                # ① 양쪽 변 중점 시각화
                cv2.circle(show_img3, pA, 4, (255, 0, 255), 1)
                cv2.circle(show_img3, pB, 4, (255, 0, 255), 1)

                # ② 선분 그리기
                cv2.line(show_img3, pA, pB, (255, 0, 255), 1)

                # ③ 중심점 계산 (두 점 평균)
                cx = (pA[0] + pB[0]) / 2
                cy = (pA[1] + pB[1]) / 2
                cv2.circle(show_img3, (int(cx), int(cy)), 5, (0, 255, 255), 1)

                # ④ 선분 기울기 계산
                dx = pB[0] - pA[0]
                dy = pB[1] - pA[1]
                if dx == 0:
                    slope = float('inf')  # 수직선
                else:
                    slope = dy / dx

                angle = math.atan2(dy, dx)
                angle = math.degrees(angle)
                if angle >= 0:
                    angle-=90
                else:
                    angle+=90
                angle = self.fold_angle(angle_deg=angle)
                # 리스트에 추가
                line_info_list.append(((cx, cy), slope, angle))
            center = None
            angle = None
            # ───────── 결과 출력 ─────────
            print("\n[Line Info List]")
            for i, (center, slope, angle) in enumerate(line_info_list):
                cx, cy = center
                print(f"{i+1:02d}. Center=({cx:.1f}, {cy:.1f}), Slope={slope:.3f}, Angle={angle:.3f}")
                center=center
                angle=angle

            if self.click_point:
                if center is not None:
                    u, v = center
                    u = u*2.0
                    v = v*1.5
                    print(u, v)

                    di = self.angles_from_pixel(depth_for_calc, u=u, v=v)
                    x = di.get('X')
                    y = di.get('Y')
                    z = di.get('Z')
                    if di is not None:
                        print(di.get('X'), di.get('Y'), di.get('Z'))
                    
                    self.indy.run(x, y, z, angle)
                    
                    self.click_point = False
                else:
                    self.click_point = False

            show = cv2.hconcat([cv2.resize(show_img1, (600,450)), cv2.resize(show_img2, (600,450)), cv2.resize(show_img3, (600,450))])
            # show = view_color
            cv2.imshow('View', show)

            if cv2.waitKey(33) & 0xFF == ord('q'):
                self.indy.close()
                cv2.destroyAllWindows()
                print("Stopping pipeline...")
                self.pipeline.stop()
                break
            

if __name__ == "__main__":
    camera().start()