#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

ARRAY_SIZE = 11
INPUTS = ['wpt', 'obst']

class Arbiter(object):
	def __init__(self):
		
		rospy.init_node('arbiter')
		
		rospy.Subscriber('lidar/cmd_vel', Twist, self.lidar_cmd_vel_cb)
		rospy.Subscriber('cmr/cmd_vel', Twist, self.cmr_cmd_vel_cb)

		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def lidar_cmd_vel_cb(self, msg):
		self.update_array(msg.data, INPUTS.index('wpt'))

	def cmr_cmd_vel_cb(self, msg): #camera command velocity
		print 'received msg'
		self.update_array(msg.data, INPUTS.index('obst'))
		print 'updated'

	def update_array(self, data, row):
		data = np.asarray(data).reshape([2, ARRAY_SIZE])
		self.vel_array[row] = data[0]
		self.turn_array[row] = data[1]

	def run(self):
		vel_sum_array = np.zeros(ARRAY_SIZE)
		turn_sum_array = np.zeros(ARRAY_SIZE)
		#vel_sum_array = np.array([1., 2, 3, 4, 5, 6, 5, 5 ,5, 2])
		#turn_sum_array = np.array([0., 0, 10, 1, 2, 3, 3, 2, 1, 1])
		
		for i in range(len(INPUTS)):
			vel_sum_array += self.vel_array[i]
			turn_sum_array += self.turn_array[i]
		
		vel = vel_sum_array.argmax()
		turn = turn_sum_array.argmax()
		msg = Twist()
		msg.linear.x = 2*vel/(ARRAY_SIZE-1)-1
		msg.angular.z = 2*turn/(ARRAY_SIZE-1)-1
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Arbiter()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()
