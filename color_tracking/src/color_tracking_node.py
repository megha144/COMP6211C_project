#!/usr/bin/env python
# Adapted from https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

import roslib
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class color_tracking:
	def	__init__(self):
		self.thresLower = (25, 50, 50)
		self.thresUpper = (35,255,255)
		
		self.image_sub = rospy.Subscriber("/vrep/image",Image,self.callback)
		self.result_pub = rospy.Publisher("/ball_pos",Point32,queue_size=1)
		self.bridge = CvBridge()
		
		self.kf = cv2.KalmanFilter(5,3)
		cv2.setIdentity(self.kf.transitionMatrix, 1.)
		self.kf.measurementMatrix = np.array([
			[1., 0., 0., 0., 0.],
			[0., 1., 0., 0., 0.],
			[0., 0., 0., 0., 1.]
			],np.float32)
		self.kf.processNoiseCov = np.array([
			[1e-3, 0, 0, 0, 0],
			[0, 1e-3, 0, 0, 0],
			[0, 0, 5., 0, 0],
			[0, 0, 0, 5., 0],
			[0, 0, 0, 0, 1e-1]
			],np.float32)
		cv2.setIdentity(self.kf.measurementNoiseCov,1e-1)
		self.ticks = cv2.getTickCount()

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

		self.precTick = self.ticks
		self.ticks = cv2.getTickCount()
		dT = (self.ticks - self.precTick) / cv2.getTickFrequency()

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

			if radius > 30:
				meas = np.array([[x],[y],[radius]],np.float32)
				self.kf.transitionMatrix = np.array([
					[1., 0, dT, 0, 0],
					[0, 1., 0, dT, 0],
					[0, 0, 1., 0, 0],
					[0, 0, 0, 1., 0],
					[0, 0, 0, 0, 1.]
					],np.float32)
				self.kf.correct(meas)

		state = self.kf.predict()
		
		p = Point32()

		x = state[0]
		p.x = x
		y = state[1]
		p.y = y
		radius = state[4]
		p.z = radius

		cv2.circle(frame, (int(x), int(y)), int(radius),
			(0, 255, 255), 2)
		cv2.circle(frame, center, 5, (0, 0, 255), -1)

		cv2.imshow("Frame", cv2.flip(frame,1))
		self.result_pub.publish(p)

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
