#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import numpy as np

ARRAY_SIZE = 11
INPUTS = ['wpt', 'obst']

class Arbiter(object):
	def __init__(self):	
		self.lidx = 0
		self.lidz = 0
		self.cmrx = "N/A"
		self.cmrz = "N/A"
		self.irx = 0
		self.irz = 0
		rospy.init_node('arbiter')
		rospy.Subscriber('lidar/cmd_vel', Twist, self.lidar_cmd_vel)
		rospy.Subscriber('cmr/cmd_vel', Twist, self.cmr_cmd_vel)
		rospy.Subscriber('ir/cmd_vel', Twist, self.ir_cmd_vel)
		rospy.Subscriber
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def lidar_cmd_vel(self, msg):
		self.lidx = msg.linear.x
		self.lidz = msg.angular.z

	def cmr_cmd_vel(self, msg): #camera command velocity
		self.cmrx = msg.linear.x
		self.cmrz = msg.angular.z
	def ir_cmd_vel(self, msg):
		self.irx = msg.linear.x
		self.irz = msg.angular.z

	def run(self):
		msg = Twist()
		if (self.cmrx!= "N/A" and self.cmrz != "N/A"):
			msg.linear.x = self.cmrx
			msg.angular.z = self.cmrz
			self.cmrx = "N/A" #after reading the camera cmds, set it to null so if it dosn't publish on the next run, follow lidar cmds
			self.cmrz = "N/A"
		else:
			msg.linear.x = self.lidx
			msg.angular.z = self.lidz

		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Arbiter()
	r = rospy.Rate(100)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()
