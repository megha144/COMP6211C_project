#!/usr/bin/env python
# Adapted from https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
from collections import deque

import roslib
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class color_tracking:
	def	__init__(self):
		self.thresLower = (25, 32, 32)
		self.thresUpper = (35,255,255)

		self.image_sub = rospy.Subscriber("/vrep/image",Image,self.callback)
		self.bridge = CvBridge()
		
	def callback(self, data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)
		self.detect(frame)
		cv2.waitKey(3)

	
	def detect(self,frame):
		# frame = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, self.thresLower, self.thresUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None

		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			if radius > 10:
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)

		cv2.imshow("Frame", cv2.flip(frame,1))

def main(args):
    ct = color_tracking()
    rospy.init_node('color_tracking_node', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
