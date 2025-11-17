import numpy as np
import cv2
from sklearn.cluster import KMeans

# xywhr     = result[0].obb.xywhr.tolist()
# xyxyxyxy  = result[0].obb.xyxyxyxy.tolist()
# 왼쪽 아래에서 부터 순서대로 오른쪽 위까지 정렬
def harvest_order(xywhr, xyxyxyxy):
    ho = list(zip(xywhr, xyxyxyxy))
    ho = sorted(ho, key=lambda x: -x[0][1])

    xywhr = [x[0] for x in ho]
    xyxyxyxy = [x[1] for x in ho]
    
    grouped_xywhr = []
    grouped_xyxyxyxy=[]
    grouped=[]
    prev_cy = None
    threshold = 70

    num=0
    group_num = -1
    index = 0
    for item in ho:

        num+=1
        cy = item[0][1]
        if prev_cy is None or abs(cy - prev_cy) > threshold:
            prev_cy = cy
            group_num+=1
            index=0
            grouped.append([])
            grouped[group_num].append(item)
        else:
            index+=1
            grouped[group_num].append(item)
        if num < 10:
            print(f"0{num} : {group_num} : {index} : {cy} : {prev_cy} : {item[0][2]} : {item[0][3]} : {item[0][4]}" )
        else:
            print(f"{num} : {group_num} : {index} : {cy} : {prev_cy} : {item[0][2]} : {item[0][3]} : {item[0][4]}" )


    for i in range(len(grouped)):
        grouped[i] = sorted(grouped[i], key=lambda x: x[0][0])

    num=0
    for item in grouped:
        grouped_xywhr.append([])
        grouped_xyxyxyxy.append([])
        for tem in item:
            grouped_xywhr[num].append(tem[0])
            grouped_xyxyxyxy[num].append(tem[1])
        num+=1

    return grouped_xywhr, grouped_xyxyxyxy


def view_item(img, xywhr, xyxyxyxy, num):

    ratio_x = img.shape[1]/640
    ratio_y = img.shape[0]/640

    t_xyxyxyxy = xyxyxyxy[num]
    t_xywhr = xywhr[num]

    t_xywhr[0], t_xywhr[2] = t_xywhr[0]*ratio_x, t_xywhr[2]*ratio_x
    t_xywhr[1], t_xywhr[3] = t_xywhr[1]*ratio_y, t_xywhr[3]*ratio_y

    for item in t_xyxyxyxy:
        item[0] = item[0]*ratio_x
        item[1] = item[1]*ratio_y

    input_pts = np.float32([t_xyxyxyxy[0], t_xyxyxyxy[1], t_xyxyxyxy[2], t_xyxyxyxy[3]])
    output_pts = np.float32([[0, 0],
                            [0, int(t_xywhr[3]) - 1],
                            [int(t_xywhr[2]) - 1, int(t_xywhr[3]) - 1],
                            [int(t_xywhr[2]) - 1, 0]])

    M = cv2.getPerspectiveTransform(input_pts, output_pts)
    out = cv2.warpPerspective(img,M,(int(t_xywhr[2]), int(t_xywhr[3])),flags=cv2.INTER_LINEAR)

    return out

def find_center(img):
    _, img = cv2.threshold(img, 160, 255, cv2.THRESH_BINARY)

    # kernel2 = np.ones((1, 2), np.uint8)
    # img = cv2.erode(img, kernel2, iterations=1)

    # kernel1 = np.ones((6, 3), np.uint8)
    # kernel2 = np.ones((1, 4), np.uint8)
    # img = cv2.dilate(img, kernel1, iterations=2)
    # img = cv2.erode(img, kernel2, iterations=2)
    # img = cv2.dilate(img, kernel1, iterations=2)
    # img = cv2.erode(img, kernel2, iterations=2)
    # img = cv2.dilate(img, kernel1, iterations=2)
    # img = cv2.erode(img, kernel2, iterations=2)
    # img = cv2.dilate(img, kernel1, iterations=2)
    # img = cv2.erode(img, kernel2, iterations=2)

    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    result = np.zeros_like(img)
    new_contours = []
    for cnt in contours:
        if len(cnt) >= 0:
            # x,y,w,h = cv2.boundingRect(cnt)
            # cv2.rectangle(result,(x,y),(x+w,y+h),(255,0,0),2)
            new_contours.append(cnt)

    centers = []
    for cnt in new_contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append([cx, cy])

    # for i in new_contours:
    #     print(i)

    # centers = np.array(centers)
    # kmeans = KMeans(n_clusters=2, random_state=0).fit(centers)
    # labels = kmeans.labels_

    # print(labels)

    result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
    # for i, cnt in enumerate(new_contours):
    #     if i < len(labels):
    #         color = (255, 0, 0) if labels[i] == 0 else (0, 255, 0)
    #         cv2.drawContours(result, [cnt], -1, color, thickness=cv2.FILLED)

    for cnt in new_contours:
        cv2.drawContours(result, [cnt], -1, (0, 255, 0), thickness=cv2.FILLED)

    return result, img




