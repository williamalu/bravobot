#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
import numpy as np

ARRAY_SIZE = 11
INPUTS = ['wpt', 'obst']

class Twist_Pub(object):
	def __init__(self):
		rospy.init_node('twist_pub')
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		
	def run(self,i):
		vels = [5, 2, 9, 7, 7]
		dirs = [5, 9, 5, 3, 8]
		vel = vels[i]
		turn = dirs[i]
		msg = Twist()
		msg.linear.x = 2*vel/(ARRAY_SIZE-1)-1
		msg.angular.z = 2*turn/(ARRAY_SIZE-1)-1
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Twist_Pub()
	r = rospy.Rate(5000)
	i = 0 #5 modes: stop, go backwards, go straight forwards, turn left, turn right
	while not rospy.is_shutdown():
		i = (i+1)%5
		main.run(i)
		r.sleep()
