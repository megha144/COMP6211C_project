#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('image_face_recognition')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as np


def draw_rectangle(img, rect):
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)


def draw_text(img, text, x, y):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)


class image_face_recognition:
    def __init__(self):
        rospy.loginfo('Initializing image face recognition')
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)
        self.laser_bool_pub = rospy.Publisher("/vrep/laser_switch", Bool, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/vrep/image", Image, self.callback)
        self.subjects = ["Barack Obama", "Avril Lavigne", "Zhang Guorong", "Legolas", "Levi Rivaille"]
        self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.path = rospy.get_param('~path', '/home/huier/Projects/comp6211c/catkin_ws/src/picture')
        rospy.loginfo(self.path)
        self.faces, self.labels = self.prepare_training_data()
        self.face_recognizer.train(self.faces, np.array(self.labels))

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

    def prepare_training_data(self):
        rospy.loginfo("Training...")
        faces = []
        labels = []
        for label in range(5):
            subject_dir_path = self.path + "/" + str(label)
            subject_images_names = os.listdir(subject_dir_path)
            for image_name in subject_images_names:
                image_path = subject_dir_path + "/" + image_name
                image = cv2.imread(image_path)
                face, rect = self.detect_face(image)
                if face is not None:
                    faces.append(face)
                    labels.append(label)
        return faces, labels

    def detect_face(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier(self.path + '/lbpcascade_frontalface.xml')
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
        if (len(faces) == 0):
            return None, None
        (x, y, w, h) = faces[0]
        return cv2.resize(gray[y:y + w, x:x + h], (50, 50)), faces[0]

    def predict(self, test_img):
        img = test_img.copy()
        face, rect = self.detect_face(img)
        coordinate = None
        if face is None:
            return img, coordinate
        label = self.face_recognizer.predict(face)
        label_text = self.subjects[label[0]]
        (x, y, w, h) = rect
        coordinate = [x + w / 2, y + h / 2]
        draw_rectangle(img, rect)
        draw_text(img, label_text, rect[0], rect[1] - 5)
        return img, coordinate


def main(args):
    ic = image_face_recognition()
    rospy.init_node('image_face_recognition', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
