#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import string
from scipy.spatial import distance
import time

global msg
msg = Twist()
# chat_pub = rospy.Publisher('chatter', String, queue_size=10)
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


def rotate(direct):
    if direct == "left":
        print("Rotating to the left")
        msg.angular.z = 0.2
    else:
        print("Rotating to the right")
        msg.angular.z = -0.2


def translate(direct):
    if direct == "forward":
        print("Moving forward")
        msg.linear.x = 0.07
    else:
        print("Moving backward")
        msg.linear.x = -0.07


def robot_stop():
    print("About to stop")
    msg.linear.x = 0
    msg.angular.z = 0


def get_depth():
    depth = 2.0
    return depth


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


def talker():
    rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(20)  # 10hz
    cap = cv2.VideoCapture(0)
    print("[INFO] loading model...")
    net = cv2.dnn.readNetFromCaffe("src/tracking/src/MobileNetSSD_deploy.prototxt.txt",
                                   "src/tracking/src/MobileNetSSD_deploy.caffemodel")
    track_box = None
    IOU_block_flag = False
    block_flag = False
    block_count = 0
    pre_center = 0
    follow_flag = False
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # chat_pub.publish(hello_str)
        vel_pub.publish(msg)
        ret, frame = cap.read()
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

                if track_box is not None:
                    robot_stop()
                    if track_box[0] < tuple([10]):
                        print("Person out of frame")
                        direction = "left"
                    elif (frame.shape[1] - track_box[2]) < tuple([10]):
                        print("Person out of frame")
                        direction = "right"
                    else:
                        if distance > 0:
                            direction = "right"
                        else:
                            direction = "left"
                    rotate(direction)
                    track_box = None
                    follow_flag = True
                continue

            else:
                if track_box is not None:
                    person_iou = list()
                    person_iou_idx = list()
                    person_idxs = list()
                    # print("Num of persons: {}".format(len(person_box)))
                    if len(person_box) == 1:
                        iou = bb_intersection_over_union(person_box[0], track_box)
                        if iou >= 0.5:
                            person_iou.append(iou)
                            person_iou_idx.append(0)
                    else:
                        for i in range(len(person_box)):
                            iou = bb_intersection_over_union(person_box[i], track_box)
                            # print("IOU: {}, {}".format(i, iou))
                            if iou >= 0.2:
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
                    # print("Area:{}, {}".format(i, area))
                    # print("Height, width: {}, {}".format(height, width))
                    delta = abs(area - 2.0)
                    if ((delta < 0.7) and (delta > -0.7)) and ((width > 100) and (height > 400)):
                        person_area.append(delta)
                if IOU_block_flag:
                    if len(person_area) > 1:
                        block_flag = True
                        print("block_flag")
                        continue
                    else:
                        IOU_block_flag = False
                if len(person_area) >= 1:
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
                z = get_depth()
                if follow_flag:
                    if 310 < center < 330:
                        robot_stop()
                        follow_flag = False
                else:
                    if z < 1.0:
                        direct = "backward"
                        translate(direct)
                    elif z > 1.8:
                        direct = "forward"
                        translate(direct)
                    else:
                        stop()
                if track_box is not None:
                    distance = center - pre_center

                cv2.rectangle(frame, track_box[:2], track_box[2:],
                              (0, 255, 0), 2)
                pre_center = center
        else:
            if block_count == 10:
                block_count = 0
                block_flag = False
                IOU_block_flag = False
                track_box_new = list()
                for i in range(len(track_box)):
                    track_box_new.append(track_box[i])
                track_box_new[0] = track_box_new[0] + int(distance * 5)
                track_box_new[2] = track_box_new[2] + int(distance * 5)
                track_box = (track_box_new[0], track_box_new[1], track_box_new[2], track_box_new[3])
            else:
                block_count += 1
                print("Block count: {}".format(block_count))
        cv2.imshow("Output", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        stop = start
        # rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
