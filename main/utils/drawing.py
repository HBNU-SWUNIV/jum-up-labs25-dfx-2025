import cv2
import numpy as np

import math

def draw_obb_box(image, points, color=(255, 0, 0), thickness=2):
    """
    points: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]] 형식
    """
    pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))  # OpenCV 포맷

    cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness)

def draw_harvest_point(image, grouped):
    num=0
    for xylist in grouped:
        for xy in xylist:
            num+=1
            x = int(xy[0])
            y = int(xy[1])
            w = xy[2]
            h = xy[3]

            # 새로운 점 계산
            if h > w:
                x2 = int(x + w * math.cos(xy[4]))
                y2 = int(y + w * math.sin(xy[4]))
            else:
                x2 = int(x + h * math.cos(xy[4]+math.radians(90)))
                y2 = int(y + h * math.sin(xy[4]+math.radians(90)))

            # 원래 점과 이동된 점 표시
            cv2.circle(image, (x, y), 5, (255, 0, 0), -1)  # 시작점 (파란색)
            cv2.circle(image, (x2, y2), 5, (0, 0, 255), -1)  # 도착점 (빨간색)
            cv2.line(image, (x, y), (x2, y2), (0, 255, 0), 2)  # 선으로 연결

def put_obb_text(image, grouped):
    num=0
    for xylist in grouped:
        for xy in xylist:
            num+=1
            x = int(xy[0])
            y = int(xy[1])

            cv2.putText(image, f"{num}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)