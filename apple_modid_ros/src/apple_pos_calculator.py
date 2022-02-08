#!/usr/bin/env python

import rospy

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class ApplePositionCalculator:

	def __init__(self):
		self.position = Vector3()
		self.l = 0

	def calculate_position(self, msg):
		
		try:
			r = R.from_quat([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
			z_h = r.as_dcm()[0:3,2]
			v_hand = np.array([msg.position.x,msg.position.y,msg.position.z])
			v_apple = v_hand+self.l*z_h

			self.position.x = v_apple[0]
			self.position.y = v_apple[1]
			self.position.z = v_apple[2]
		except:
			print("Did not recieve valid pose information")

	def update_l(self,msg):
		self.l = msg.data


if __name__ == '__main__':

    rospy.init_node('apple_pos_publisher')

    calculator = ApplePositionCalculator()

    pose_subscriber = rospy.Subscriber('ur5e_pose', Pose, calculator.calculate_position)
    l_subscriber = rospy.Subscriber('l_estimate', Float64, calculator.update_l)

    position_publisher = rospy.Publisher('apple_position_estimate', Vector3, queue_size=10)

    while not rospy.is_shutdown():

        position_publisher.publish(calculator.position)
