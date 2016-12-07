#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

ARRAY_SIZE = 11
INPUTS = ['wpt', 'obst']

class Arbiter(object):
	def __init__(self):	
		self.x = 0
		self.z = 0
		rospy.init_node('arbiter')
		rospy.Subscriber('lidar/cmd_vel', Twist, self.lidar_cmd_vel)
		rospy.Subscriber('cmr/cmd_vel', Twist, self.cmr_cmd_vel)
		rospy.Subscriber('ir/cmd_vel', Twist, self.ir_cmd_vel)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def lidar_cmd_vel_cb(self, msg):
		self.x += msg.linear.x
		self.z += msg.linear.z
	def cmr_cmd_vel_cb(self, msg): #camera command velocity
		self.x += msg.linear.x
		self.z += msg.linear.z
	def ir_cmd_vel(self, msg):
		self.x += msg.linear.x
		self.z += msg.linear.z

	def run(self):
		msg = Twist()
		msg.linear.x = self.x
		msg.angular.z = self.z
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Arbiter()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()
