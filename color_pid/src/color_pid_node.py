#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Point32, Twist

class color_pid:
	def __init__(self):
		self.result_sub = rospy.Subscriber("/ball_pos",Point32,self.callback)
		self.cmd_pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=1)

		self.linear_err = 0
		self.angular_err = 0
		self.prev_linear_err = 0
		self.prev_angular_err = 0
		self.linear_Kp = rospy.get_param('linear_Kp',5.0)
		self.linear_Kd = rospy.get_param('linear_Kd',1.0)
		self.angular_Kp = rospy.get_param('angular_Kp',3.0)
		self.angular_Kd = rospy.get_param('angular_Kd',0.5)

	def calc(self):
		vx = self.linear_Kp * self.linear_err + self.linear_Kd * (self.linear_err - self.prev_linear_err)
		vw = self.angular_Kp * self.angular_err + self.angular_Kd * (self.angular_err - self.prev_angular_err)

		self.prev_linear_err = self.linear_err
		self.prev_angular_err = self.angular_err

		vel_msg = Twist()
		vel_msg.linear.x = vx
		vel_msg.angular.z = vw
		self.cmd_pub.publish(vel_msg)

	def callback(self,p):
		# r_sp = 140, x_sp = 256		
		self.linear_err = p.z + 140
		self.angular_err = p.y - 256
		self.calc()

if __name__=="__main__":
	rospy.init_node('color_pid')
	pid = color_pid()

	rospy.spin()
	
