#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
from cv_bridge import CvBridge
from sklearn.cluster import KMeans
from scipy.spatial import distance as eu


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


class Image_c:
    def __init__(self):
        self.my_image = None
        self.my_depth_image = None
        self.frame_sub = rospy.Subscriber('camera/color/image_raw', Image, self.color_img_callback)
        self.depth_frame_sub = rospy.Subscriber('camera/aligned_depth_to_color/image_raw', Image,
                                                self.depth_img_callback)

    def color_img_callback(self, realsense_img):
        self.my_image = cv2.cvtColor(bridge.imgmsg_to_cv2(realsense_img, desired_encoding='passthrough'),
                                     cv2.COLOR_BGR2RGB)
        # print("Image_raw")
        # print(self.my_image.shape)

    def depth_img_callback(self, realsense_depth_img):
        self.my_depth_image = cv2.cvtColor(bridge.imgmsg_to_cv2(realsense_depth_img, desired_encoding='passthrough'),
                                           cv2.COLOR_BGR2RGB)
        # print("Depth")
        # print(self.my_depth_image.shape)
        # print(self.my_depth_image[5, 5])

    def get_depth(self, x, y):
        depth = self.my_depth_image[y, x]
        return depth[0]


class Following:
    def __init__(self):
        print(cv2.__version__)
        self.msg = Twist()
        self.frame = None
        self.frame_h = 0
        self.frame_w = 0
        self.depth = 1.5
        self.track_box = None
        self.IOU_block_flag = False
        self.block_flag = False
        self.start = 0
        self.stop = 0
        self.center = 0
        self.pre_center = 0
        self.distance = 1.5
        self.follow_flag = False
        self.reappear_flag = True
        self.min_distance = 1.2
        self.max_distance = 1.5
        self.block_count = 0
        self.angular_speed = 0.2
        self.linear_speed = 0.1
        self.retrieve_threshold = 80
        self.rotate_direction = None
        self.feature = None
        self.get_feature_flag = False
        self.tb_from_tracker_flag = False
        self.person_wait_count = 0
        self.max_person_wait_count = 0
        self.get_random =False
        self.absence_count = 0

    def rotate(self):
        if self.rotate_direction == "left":
            print("Rotating to the left")
            self.msg.angular.z = self.angular_speed
            vel_pub.publish(self.msg)
        else:
            print("Rotating to the right")
            self.msg.angular.z = -self.angular_speed
            vel_pub.publish(self.msg)

    def translate(self, direction):
        if direction == "forward":
            print("Moving forward")
            self.msg.linear.x = self.linear_speed
            vel_pub.publish(self.msg)
        else:
            print("Moving backward")
            self.msg.linear.x = -self.linear_speed
            vel_pub.publish(self.msg)

    def robot_stop(self):
        print("Stop")
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        vel_pub.publish(self.msg)

    def follow(self):     
        if int(self.track_box[0]) < 10:
            self.robot_stop()
            print("Person out of frame")
            self.rotate_direction = "left"
            self.rotate()
            self.follow_flag = True
            self.reappear_flag = False
        elif int(self.frame_w - self.track_box[2]) < 10:
            self.robot_stop()
            print("Person out of frame")
            self.rotate_direction = "right"
            self.rotate()
            self.follow_flag = True
            self.reappear_flag = False
        else:
            if self.absence_count == 20:
                if self.center < 310:
                    self.direction = "left"
                else:
                    self.direction = "right"
                self.angular_speed = 0.1
                print("Finding person")
                self.rotate()
            else:
                self.absence_count += 1
        # else:
        #     # if distance > 0:
        #     #     self.direction = "right"
        #     # else:
        #     #     self.direction = "left"
        #     print("Using tracker")
        #     initBB = (self.track_box[0], self.track_box[1], self.frame_w, self.frame_h)
        #     tracker.init(self.frame, initBB)
        #     return True
        return False

    def get_feature(self, box):
        box = box[:-int(box.shape[0] / 3), int(box.shape[1] / 4):-int(box.shape[1] / 4)]
        if not self.get_feature_flag:
            cv2.imshow("test", box)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()
        image = cv2.cvtColor(box, cv2.COLOR_BGR2RGB)
        image = image.reshape((image.shape[0] * image.shape[1], 3))
        clt = KMeans(3)
        clt.fit(image)
        mot = np.where(clt.labels_ == 0, 1, 0)
        hai = np.where(clt.labels_ == 1, 1, 0)
        ba = np.where(clt.labels_ == 2, 1, 0)
        compare = list()
        compare.append(np.sum(mot))
        compare.append(np.sum(hai))
        compare.append(np.sum(ba))
        position = np.array(compare)
        position = np.argmax(position)
        clt.cluster_centers_= clt.cluster_centers_*0.75
        clt.cluster_centers_[position, :] = clt.cluster_centers_[position, :]/0.75
        if not self.get_feature_flag:
            print("Original Feature: {}".format(clt.cluster_centers_))
            feature = clt.cluster_centers_.flatten()
            return feature
        print("New Feature: {}".format(clt.cluster_centers_))
        feature = np.empty((6, 9))
        feature[0, :] = clt.cluster_centers_[(0, 1, 2), :].flatten()
        feature[1, :] = clt.cluster_centers_[(0, 2, 1), :].flatten()
        feature[2, :] = clt.cluster_centers_[(1, 0, 2), :].flatten()
        feature[3, :] = clt.cluster_centers_[(1, 2, 0), :].flatten()
        feature[4, :] = clt.cluster_centers_[(2, 0, 1), :].flatten()
        feature[5, :] = clt.cluster_centers_[(2, 1, 0), :].flatten()
        return feature

    def get_difference(self, bounding_box):
        box_arr = np.array(bounding_box)
        box_arr = np.where(box_arr < 0, 0, box_arr)
        box = self.frame[box_arr[1]:box_arr[3], box_arr[0]:box_arr[2]]
        feature_new = self.get_feature(box)
        difference = (feature_new - self.feature) * (feature_new - self.feature)
        difference = np.sqrt(np.sum(difference, axis=1))
        difference = np.min(difference)
        return difference

    def person_follow(self):
        if self.follow_flag:
            if 300 < self.center < 330:
                # self.robot_stop()
                self.msg.angular.z = 0
                vel_pub.publish(self.msg)
                self.follow_flag = False
                print("Person reappeared in center")
        else:
            if self.depth < self.min_distance:
                direct = "backward"
                self.translate(direct)
            elif self.depth > self.max_distance:
                direct = "forward"
                self.translate(direct)
            else:
               self.msg.linear.x = 0
               vel_pub.publish(self.msg)
            if self.center < int(self.frame_w / 4):
                print("Person is more at the left >>> move left")
                self.rotate_direction = "left"
                self.rotate()
                self.follow_flag = True
            elif self.center > 3 * int(self.frame_w / 4):
                print("Person is more at the right >>> move right")
                self.rotate_direction = "right"
                self.rotate()
                self.follow_flag = True
            if not self.get_feature_flag:
                track_box_arr = np.array(self.track_box)
                track_box_arr = np.where(track_box_arr < 0, 0, track_box_arr)
                track_box = (track_box_arr[0], track_box_arr[1], track_box_arr[2], track_box_arr[3])
                box = self.frame[track_box[1]:track_box[3], track_box[0]:track_box[2]]
                self.feature = self.get_feature(box)
                self.get_feature_flag = True

    def tracking(self):
        while not rospy.is_shutdown():
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.robot_stop()
                vel_pub.publish(self.msg)
                break
            vel_pub.publish(self.msg)
            self.frame = image_c.my_image
            # ret, self.frame = cap.read()
            if self.frame is None:
                print("No image")
                continue
            self.start = time.time()
            print("FPS = {}".format(int(1 / (self.start - self.stop))))
            self.stop = self.start
            if not self.block_flag:
                (self.frame_h, self.frame_w) = self.frame.shape[:2]
                blob = cv2.dnn.blobFromImage(cv2.resize(self.frame, (300, 300)), 0.007843,
                                             (300, 300), 127.5)  # local
                net.setInput(blob)
                detections = net.forward()
                person_box = list()
                for i in np.arange(0, detections.shape[2]):
                    confidence = detections[0, 0, i, 2]
                    idx = int(detections[0, 0, i, 1])
                    if idx == 15:
                        if confidence > 0.7:
                            box = detections[0, 0, i, 3:7] * np.array(
                                [self.frame_w, self.frame_h, self.frame_w, self.frame_h])
                            (x1, y1, x2, y2) = box.astype("int")
                            person_box.append((x1, y1, x2, y2))
                if len(person_box) < 1:
                    print("No person detected")
                    self.msg.linear.x = 0
                    vel_pub.publish(self.msg)
                    if not self.reappear_flag:
                        continue
                    if self.track_box is not None:
                        self.block_flag = self.follow()
                        self.tb_from_tracker_flag = True
                    continue
                else:
                    if (self.track_box is not None) and (not self.reappear_flag):
                        # print("First appear after absence")
                        # print("{} persons detected".format(len(person_box)))
                        if not self.get_random:
                            person_box_temp = list()
                            for i in range(len(person_box)):
                                temp_center = ((person_box[i][2] - person_box[i][0])/2) + person_box[i][0]
                                if (self.rotate_direction == "left" and temp_center < 400) or (self.rotate_direction == "right" and temp_center > 240):
                                    person_box_temp.append(person_box[i])
                            person_box = None
                            person_box = person_box_temp
                            print("{} persons eligible".format(len(person_box)))
                    elif (self.track_box is not None) and self.reappear_flag:
                        # print(track_box)
                        person_iou = list()
                        person_iou_idx = list()
                        person_idxs = list()
                        # print("Num of persons: {}".format(len(person_box)))
                        iou_threshold = 0.2
                        if not self.tb_from_tracker_flag:
                            iou_threshold = 0.1
                        if len(person_box) == 1:
                            iou = bb_intersection_over_union(person_box[0], self.track_box)
                            if iou >= iou_threshold:
                                person_iou.append(iou)
                                person_iou_idx.append(0)
                        else:
                            for i in range(len(person_box)):
                                iou = bb_intersection_over_union(person_box[i], self.track_box)
                                # print("IOU: {}, {}".format(i, iou))
                                if iou >= iou_threshold:
                                    person_iou.append(iou)
                                    person_iou_idx.append(i)
                        if len(person_iou) < 1:
                            print("No person detected near old track box")
                            self.msg.linear.x = 0
                            vel_pub.publish(self.msg)
                            if not self.reappear_flag:
                                continue
                            self.block_flag = self.follow()
                            continue
                        if  self.absence_count > 0:
                            self.absence_count = 0
                            self.angular_speed = 0.2
                            if self.absence_count == 20:
                                self.robot_stop()
                        person_box = [person_box[k] for k in person_iou_idx]
                        if len(person_iou) == 1:
                            person_iou_max = person_box[0]
                        elif len(person_iou) > 1:
                            person_idx = np.argmax(person_iou)
                            person_iou_max = person_box[person_idx]
                            person_idxs.append(person_idx)
                            person_iou[person_idx] = -1
                            person_idxs.append(np.argmax(person_iou))
                            person_box = [person_box[k] for k in person_idxs]
                            if self.reappear_flag:
                                self.IOU_block_flag = True
                                print("IOU_block_flag")
                    person_area = list()
                    if self.reappear_flag:
                        for i in range(len(person_box)):
                            width = person_box[i][2] - person_box[i][0]
                            height = person_box[i][3] - person_box[i][1]
                            area = float(height / width)
                            # print("Area is {}".format(area))
                            # print(person_box[i])
                            if self.IOU_block_flag:
                                area_threshold = 1.0
                            else:
                                area_threshold = 2.0
                            delta = abs(area - area_threshold)
                            # if ((delta < 0.7) and (delta > -0.7)) and ((width > 100) and (height > 400)):
                            if (delta < 0.7) and (delta > -0.7):
                                person_area.append(delta)
                        # person_area = person_box
                        # print("{} persons are alike a body form".format(len(person_area)))                    
                        if self.IOU_block_flag:
                            print("2 person near old track box")
                            self.IOU_block_flag = False
                            if len(person_area) > 1:
                                self.follow_flag = False
                                self.robot_stop()
                                compare_array = np.empty(len(person_area))
                                for i in range(len(person_area)):
                                    compare_array[i] = self.get_difference(person_box[i])
                                difference = np.min(compare_array)
                                print("Distance is {}".format(distance))
                                if difference > self.retrieve_threshold - 10:
                                    initBB = (self.track_box[0], self.track_box[1], self.frame_w, self.frame_h)
                                    tracker.init(self.frame, initBB)
                                    self.block_flag = True
                                    continue
                                self.track_box = person_box[np.argmin(compare_array)]
                        else:
                            if len(person_area) >= 1:
                                self.track_box = person_box[np.argmin(person_area)]
                            else:
                                if self.track_box is not None:
                                    self.track_box = person_iou_max
                                else: 
                                    continue
                    else:
                        # print("Length of ther person_box during out of frame is {}".format(len(person_area)))
                        if len(person_box) == 1:
                            # self.robot_stop()
                            if self.get_random:
                                print("Get random person")
                                self.get_random = False
                                self.track_box = person_box[0]
                            else:
                                # print("Feature is {}".format(self.feature))
                                difference = self.get_difference(person_box[0])
                                print("Difference is {}".format(difference))
                                # print("Difference is {}".format(difference))
                                if difference <= self.retrieve_threshold:
                                    print("SAME PERSON reappeared")
                                    self.robot_stop()
                                    self.track_box = person_box[0]
                                    # self.angular_speed = 0.2
                                    self.person_wait_count = 0
                                    self.max_person_wait_count = 0
                                else:
                                    print("Not the same person")
                                    self.person_wait_count += 1
                                    # print("Person wait count is {}".format(self.person_wait_count))
                                    if self.person_wait_count == 2:
                                        self.person_wait_count = 0
                                        self.max_person_wait_count += 1
                                        # print("MAX Person wait count is {}".format(self.max_person_wait_count))
                                        if self.max_person_wait_count == 4:
                                            self.max_person_wait_count = 0
                                            self.get_random = True
                                            self.robot_stop()
                                        else: 
                                            self.rotate()
                                    else:       
                                        self.robot_stop()
                                    continue
                            self.reappear_flag = True
                            self.follow_flag = False
                        elif len(person_box) > 1:
                            print("Two persons found after absence")
                            compare_array = np.empty(len(person_box))
                            for i in range(len(person_box)):
                                # cv2.rectangle(self.frame, person_box[i][:2], person_box[i][2:], (0, 255, 0),
                                #               2)
                                compare_array[i] = self.get_difference(person_box[i])
                            if not self.get_random:
                                difference = np.min(compare_array)
                                print("Difference is {}".format(difference))
                                if difference > self.retrieve_threshold:
                                    continue
                            print("Get random person")
                            self.track_box = person_box[np.argmin(compare_array)]
                            self.max_person_wait_count = 0
                            self.person_wait_count = 0
                            if self.get_random:
                                self.get_random = False
                            self.angular_speed = 0.2
                            self.robot_stop()
                            self.reappear_flag = True
                            self.follow_flag = False
                        else:
                            continue                                                
                    cv2.rectangle(self.frame, self.track_box[:2], self.track_box[2:], (0, 255, 0), 2)
                    w = self.track_box[2] - self.track_box[0]
                    h = self.track_box[3] - self.track_box[1]
                    self.center = self.track_box[0] + (w / 2)
                    self.depth = (image_c.get_depth(int(self.center), int(self.track_box[1] + (h / 4))) / 1000.0)
                    # print("Depth: {}".format(self.depth))
                    self.person_follow()
                    self.distance = self.center - self.pre_center
                    self.pre_center = self.center
                    # self.tb_from_tracker_flag = False
            else:
                print("Block count {}".format(self.block_count))
                if self.block_count == 5:
                    self.block_count = 0
                    self.block_flag = False
                    print("Tracker found target person")
                else:
                    self.block_count += 1
                (success, tracker_box) = tracker.update(self.frame)
                if success:
                    (x, y, w, h) = [int(v) for v in tracker_box]
                    self.track_box = (x, y, x + w, y + h)
                    cv2.rectangle(self.frame, self.track_box[:2], self.track_box[2:],
                                  (0, 255, 0), 2)
                    self.center = self.track_box[0] + (w / 2)
                    self.depth = (image_c.get_depth(int(self.center), int(self.track_box[1] + (h / 4))) / 1000.0)
                    # print("Depth: {}".format(self.depth))
                    self.distance = self.center - self.pre_center
                    self.pre_center = self.center
                    self.person_follow()
            cv2.imshow("Output", self.frame)
            # rate.sleep()


if __name__ == '__main__':
    try:
        print("[INFO] loading model...")
        bridge = CvBridge()
        tracker = cv2.TrackerCSRT_create()
        net = cv2.dnn.readNetFromCaffe("src/tracking/src/MobileNetSSD_deploy.prototxt.txt",
                                       "src/tracking/src/MobileNetSSD_deploy.caffemodel")
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        # cap = cv2.VideoCapture(0)
        image_c = Image_c()
        following = Following()
        rospy.init_node('talker', anonymous=True)
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        following.tracking()
    except rospy.ROSInterruptException:
        pass
