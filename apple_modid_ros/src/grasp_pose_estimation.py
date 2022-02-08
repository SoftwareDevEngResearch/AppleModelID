#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64

class GraspPoseEstimator:

	def __init__(self):

		self.disagreement = 0
		self.estimate = 0

	def estimate_distance(self,msg):
		
		f_x = msg.wrench.force.x		
		f_y = msg.wrench.force.y
		t_x = msg.wrench.torque.x		
		t_y = msg.wrench.torque.y

		
		est1 = -1*t_x/f_y
		est2 = t_y/f_x

		self.disagreement = abs(est1-est2)
		self.estimate = (est1+est2)/2

if __name__ == '__main__':

    rospy.init_node('grasp_pose_estimator')

    pose_estimator = GraspPoseEstimator()

    joint_subscriber = rospy.Subscriber('wrench', WrenchStamped, pose_estimator.estimate_distance)

    grasp_pose_publisher = rospy.Publisher('l_estimate', Float64, queue_size=10)
    l_uncertainty_publisher = rospy.Publisher('l_disagreement', Float64, queue_size=10)

    while not rospy.is_shutdown():

        grasp_pose_publisher.publish(pose_estimator.estimate)
	l_uncertainty_publisher.publish(pose_estimator.disagreement)

