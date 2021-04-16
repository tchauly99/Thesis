import numpy as np
import cv2
import configure
import os
import time
from scipy.spatial import distance

def bb_intersection_over_union(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou


# hello = [1, 2, 3, 4, 5, 6, 7]
# hello = [hello[i] for i in [5, 2, 6, 1]]
# print(hello)

print("[INFO] loading model...")
print(os.path.exists(configure.CAFFE_PROTOTEXT_PATH))
net = cv2.dnn.readNetFromCaffe(configure.CAFFE_PROTOTEXT_PATH, configure.CAFFE_MODEL_PATH)
# tracker = cv2.TrackerGOTURN_create()
tracker = cv2.TrackerKCF_create()
capture = cv2.VideoCapture(0)
count = 0
initBB = None
track_box = None
IOU_block_flag = False
block_flag = False
block_count = 0
center = 0
pre_center = 0
stop = 0
while True:
    ret, frame = capture.read()
    if frame is None:
        break
    start = time.time()
    if not block_flag:
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843,
                                     (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()
        person_box = list()
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.3:
                idx = int(detections[0, 0, i, 1])
                if idx == 15:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (x1, y1, x2, y2) = box.astype("int")
                    person_box.append((x1, y1, x2, y2))
        if len(person_box) < 1:
            continue
        else:
            if track_box is not None:
                person_iou = list()
                person_iou_idx = list()
                person_idxs = list()
                print("Num of persons: {}".format(len(person_box)))
                if len(person_box) == 1:
                    iou = bb_intersection_over_union(person_box[0], track_box)
                    if iou >= 0.5:
                        person_iou.append(iou)
                        person_iou_idx.append(0)
                else:
                    for i in range(len(person_box)):
                        iou = bb_intersection_over_union(person_box[i], track_box)
                        print("IOU: {}, {}".format(i, iou))
                        if iou >= 0.1:
                            person_iou.append(iou)
                            person_iou_idx.append(i)
                if len(person_iou) < 1:
                    continue
                person_box = [person_box[k] for k in person_iou_idx]
                if len(person_iou) == 1:
                    person_iou_max = person_box[0]
                elif len(person_iou) > 1:
                    print("2 person near old trackbox")
                    person_idx = np.argmax(person_iou)
                    person_iou_max = person_box[person_idx]
                    person_idxs.append(person_idx)
                    person_iou[person_idx] = -1
                    person_idxs.append(np.argmax(person_iou))
                    person_box = [person_box[k] for k in person_idxs]
                    IOU_block_flag = True
                    print("IOU_block_flag")
            person_area = list()
            for i in range(len(person_box)):
                width = person_box[i][2] - person_box[i][0]
                height = person_box[i][3] - person_box[i][1]
                area = float(height / width)
                print("Area:{}, {}".format(i, area))
                print("Height, width: {}, {}".format(height, width))
                delta = abs(area - 2.0)
                if ((delta < 0.7) and (delta > -0.7)) and ((width > 100) and (height > 400)):
                    person_area.append(delta)
            if len(person_area) >= 1:
                if len(person_area) == 1:
                    if IOU_block_flag:
                        block_flag = True
                        print("block_flag")
                        continue
                # print("Person legs")
                (x1, y1, x2, y2) = person_box[np.argmin(person_area)]
                track_box = (x1, y1, x2, y2)
                # cv2.rectangle(frame, track_box[:2], track_box[2:],
                #               (0, 255, 0), 2)
            else:
                if track_box is not None:
                    track_box = person_iou_max
                    # cv2.rectangle(frame, track_box[:2], track_box[2:],
                    #               (0, 255, 0), 2)
                else:
                    continue
            w = track_box[2] - track_box[0]
            h = track_box[3] - track_box[1]
            center = track_box[0] + (w / 2)
            if track_box is not None:
                distance = center-pre_center

            cv2.rectangle(frame, track_box[:2], track_box[2:],
                          (0, 255, 0), 2)
            pre_center = center
    else:
        if block_count == 20:
            block_count = 0
            block_flag = False
            IOU_block_flag = False
            track_box_new = list()
            for i in range(len(track_box)):
                track_box_new.append(track_box[i])
            track_box_new[0] = track_box_new[0] + int(distance*10)
            track_box_new[2] = track_box_new[2] + int(distance*10)
            track_box = (track_box_new[0], track_box_new[1], track_box_new[2], track_box_new[3])
        else:
            block_count += 1
            print("Block count: {}".format(block_count))
    cv2.imshow("Output", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    stop = start
