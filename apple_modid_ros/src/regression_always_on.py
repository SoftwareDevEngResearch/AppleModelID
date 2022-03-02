#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool


if __name__ == '__main__':

	rospy.init_node('constant_regression')

	publisher = rospy.Publisher('run_regression', Bool, queue_size=10)

	while not rospy.is_shutdown():

		publisher.publish(True)
