#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('image_sift')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as np


class image_sift:
    def __init__(self):
        rospy.loginfo('Initializing image sift')
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)
        self.laser_bool_pub = rospy.Publisher("/vrep/laser_switch", Bool, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/vrep/image", Image, self.callback)
        self.subjects = ["Barack Obama", "Avril Lavigne", "Zhang Guorong", "Legolas", "Levi Rivaille"]
        self.path = rospy.get_param('~path', '/home/huier/Projects/comp6211c/catkin_ws/src/picture/orig')
        self.k = rospy.get_param('~k', 2)
        self.sift = cv2.SIFT()
        self.bf = cv2.BFMatcher()
        self.kp_list = []
        self.des_list = []

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

        flipped_image = cv2.flip(cv_image, 1)
        final_image, center_coordinate = self.predict(flipped_image)
        cv2.imshow("Image window", final_image)
        cv2.waitKey(3)

        # self.laser_bool_pub.publish(False)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(final_image, "bgr8"))
        except CvBridgeError as e:
            rospy.loginfo(e)

    def train(self):
        for label in range(len(self.subjects)):
            image_path = self.path + "/" + str(label) + ".jpg"
            img = cv2.imread(image_path, 0)
            kp, des = self.sift.detectAndCompute(img, None)
            self.kp_list.append(kp)
            self.des_list.append(des)

    def match(self, test_image):
        test_kp, test_des = self.sift.detectAndCompute(test_image, None)
        matching_result = []
        for i in range(5):
            matches = self.bf.knnMatch(test_des, self.des_list[i], k=self.k)
            good = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    good.append([m])
            matching_result.append(len(good))

        if max(matching_result)
