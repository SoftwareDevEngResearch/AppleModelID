#!/usr/bin/env python

import numpy as np
import rospy
from applehand.msg import ImuNaive
from std_msgs.msg import Float64

class ImuFilter:

	def __init__(self):

		self.memory = []
		self.hold = None
		self.threshold = 50 #tuned parameter for detecting bounces/spikes
		self.filtered_value = 0.0
	
	def add_value(self,value):

		self.memory.append(value)
		if len(self.memory) > 5:
			self.memory = self.memory[-5:]

	def change_suspicious(self, val1, val2):

		return abs(val1 - val2) > self.threshold


	#returns true if a point is likely a spike, so that the value will not be added
	#removes spikes if detected
	def check_for_spike(self, value):

		if len(self.memory) == 0: #it's not a spike if it's the first point registered
			return False

		elif self.hold is None:
			
				if self.change_suspicious(value,self.memory[-1]):
					return True
				else:
					return False
			
		else:
			if self.change_suspicious(value,self.hold):
				self.hold = None
			return False

	def process_point(self,value):
		
		is_potential_spike = self.check_for_spike(value)
		if is_potential_spike:
			self.hold = value
		else:
			if self.hold is not None:
				self.add_value(self.hold)
				self.hold = None
			self.add_value(value)

	def filter_update(self, msg):
		value = msg.angular_velocity.x
		self.process_point(value)
		self.filtered_value = np.mean(self.memory)

if __name__ == '__main__':

    rospy.init_node('imu_filter')

    imu_filter = ImuFilter()

    imu_subscriber = rospy.Subscriber('imu_topic', ImuNaive, imu_filter.filter_update)

    filter_publisher = rospy.Publisher('filtered_imu_topic', Float64, queue_size=10)

    while not rospy.is_shutdown():

        filter_publisher.publish(imu_filter.filtered_value)

