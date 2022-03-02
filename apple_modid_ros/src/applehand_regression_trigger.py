#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool

class GraspAdjustmentReader:

	def __init__(self):
		self.on = False

	def opposites(self,msg):
		self.on = not msg.data


if __name__ == '__main__':

	rospy.init_node('applehand_regression_trigger')

	gar = GraspAdjustmentReader()

	publisher = rospy.Publisher('run_regression', Bool, queue_size=10)
	subscriber = rospy.Subscriber('grasp_adjusting', Bool, gar.opposites)

	while not rospy.is_shutdown():

		publisher.publish(gar.on)
