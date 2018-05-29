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
from sensor_msgs.msg import PointCloud


class image_sift:
    def __init__(self):
        rospy.loginfo('Initializing image sift')
        self.image_pub = rospy.Publisher("/processed_image_sift", Image, queue_size=1)
        self.laser_bool_pub = rospy.Publisher("/vrep/laser_switch", Bool, queue_size=1)
        self.point_cloud_pub = rospy.Publisher("/image_point_cloud", PointCloud, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/vrep/image", Image, self.callback)
        self.subjects = ["Barack Obama", "Avril Lavigne", "Zhang Guorong", "Legolas", "Levi Rivaille"]
        self.path = rospy.get_param('~path', '/home/huier/Projects/comp6211c/catkin_ws/src/picture/orig')
        self.k = rospy.get_param('~k', 2)
        self.threshold = rospy.get_param('~threshold', 10)
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.bf = cv2.BFMatcher()
        self.kp_list = []
        self.des_list = []
        self.train()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

        flipped_image = cv2.flip(cv_image, 1)
        sift_image, kp, max_index = self.match(flipped_image)
        cv2.imshow("Sift image", sift_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(sift_image, "bgr8"))
        except CvBridgeError as e:
            rospy.loginfo(e)

    def train(self):
        for label in range(5):
            image_path = self.path + "/" + str(label) + ".jpg"
            img = cv2.imread(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kp, des = self.sift.detectAndCompute(gray, None)
            self.kp_list.append(kp)
            self.des_list.append(des)

    def match(self, test_image):
        img = test_image.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        test_kp, test_des = self.sift.detectAndCompute(gray, None)
        matching_result = []
        for i in range(5):
            matches = self.bf.knnMatch(test_des, self.des_list[i], k=self.k)
            good = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    good.append([m])
            matching_result.append(len(good))

        max_index = None
        show_kp = None
        if max(matching_result) > self.threshold:
            max_index = matching_result.index(max(matching_result))
            rospy.loginfo('Now we see ' + self.subjects[max_index])
            matches = self.bf.knnMatch(test_des, self.des_list[max_index], k=self.k)
            index = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    index.append(m.queryIdx)
            show_kp = [test_kp[i] for i in index]
            cv2.drawKeypoints(img, show_kp, img,
                              flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return img, show_kp, max_index


def main(args):
    hehe = image_sift()
    rospy.init_node('image_sift', anonymous=True)
    rospy.loginfo('image sift node starting...')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
