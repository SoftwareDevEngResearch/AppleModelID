#!/usr/bin/env python

import rospy

import numpy as np

from geometry_msgs.msg import Vector3, WrenchStamped
from std_msgs.msg import Bool

class AppleRegression:

	def __init__(self):

		self.k_est = {'x':0,'y':0,'z':0}
		self.param_est = {'x':0,'y':0,'z':0}


		self.forces = {'x':[],'y':[],'z':[]}
		self.positions = {'x':[],'y':[],'z':[]}

		self.chunk_size = 100
		self.running = False


	def update_force(self,msg):
		if self.running:
			self.forces['x'].append(msg.wrench.force.x)
			self.forces['y'].append(msg.wrench.force.y)
			self.forces['z'].append(msg.wrench.force.z)
		
			self.size_dict(self.forces)
			


	def update_position(self,msg):

		if self.running:
			self.positions['x'].append(msg.x)
			self.positions['y'].append(msg.y)
			self.positions['z'].append(msg.z)

			self.size_dict(self.positions)


	def size_dict(self, dict_to_size):

		for key in dict_to_size:
			if len(dict_to_size[key]) > self.chunk_size:
				dict_to_size[key] = dict_to_size[key][-1*self.chunk_size:]


	def run_regression(self):
		
		for key in ['x','y','z']:
			force_data = self.forces[key]
			position_data = self.positions[key]
			if len(force_data) == self.chunk_size and len(position_data) == self.chunk_size:

				minus_k, k_q0 = np.polyfit(position_data, force_data, 1)
				k_est = -1*minus_k
				q0_est = k_q0/k_est

				self.param_est[key] = q0_est
				self.k_est[key] = k_est

	def get_publishable_values(self):
		
		k_estimate = Vector3()
		k_estimate.x = self.k_est['x']
		k_estimate.y = self.k_est['y']
		k_estimate.z = self.k_est['z']

		joint_loc = Vector3()
		joint_loc.x = self.param_est['x']
		joint_loc.y = self.param_est['y']
		joint_loc.z = self.param_est['z']

		return k_estimate, joint_loc

	#update regression state, but only if it changes
	def check_regression(self, msg):

		if msg.data:
			if self.running = False:
				self.running = True
		else:
			if self.running = True:
				self.running = False
				self.forces = {'x':[],'y':[],'z':[]}
				self.positions = {'x':[],'y':[],'z':[]}


if __name__ == '__main__':

	rospy.init_node('apple_regression')

	print("Output to terminal works!") #!!! debugging

	regression_object = AppleRegression()

	wrench_subscriber = rospy.Subscriber('wrench', WrenchStamped, regression_object.update_force)
	apple_position_subscriber = rospy.Subscriber('apple_position_estimate', Vector3, regression_object.update_position)
	regression_on_checker = rospy.Subscriber('regression_trigger_topic', Bool)

	k_publisher = rospy.Publisher('k_estimate', Vector3, queue_size=10)
	absc_layer_publisher = rospy.Publisher('abscission_joint_location', Vector3, queue_size=10)

	while not rospy.is_shutdown():

		k_estimate, joint_loc = regression_object.get_publishable_values()

		k_publisher.publish(k_estimate)
		absc_layer_publisher.publish(joint_loc)


