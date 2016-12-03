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

	def run(self):
		vel_sum_array = np.zeros(ARRAY_SIZE)
		turn_sum_array = np.zeros(ARRAY_SIZE)
		#vel_sum_array = np.array([1., 2, 3, 4, 5, 6, 5, 5 ,5, 2])
		#turn_sum_array = np.array([0., 0, 10, 1, 2, 3, 3, 2, 1, 1])
		
		vel = random.randint(0,11)
		turn = random.randint(0,11)
		msg = Twist()
		msg.linear.x = 2*vel/(ARRAY_SIZE-1)-1
		msg.angular.z = 2*turn/(ARRAY_SIZE-1)-1
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Twist_Pub()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()
