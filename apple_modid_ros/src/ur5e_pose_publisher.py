#!/usr/bin/env python

import rospy

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class PosePublisher:
	def __init__(self):
		self.dh_params = [[0,0,0.1625,np.pi/2],[0,-0.425,0,0],[0,-0.3922,0,0],[0,0,0.1333,np.pi/2],[0,0,0.0997,-1*np.pi/2],[0,0,0.09996,0]]
		self.curr_pose = Pose()
	
	def dh_to_transform(self,theta, theta0, a, d, alpha):
 	       
		theta = theta+theta0
		
		T = np.zeros([4,4])
		
		#first row
		T[0,0] = np.cos(theta)
		T[0,1] = -1*np.sin(theta)*np.cos(alpha)
		T[0,2] = np.sin(theta)*np.sin(alpha)
		T[0,3] = np.cos(theta)*a
		
		#second row
		T[1,0] = np.sin(theta)
		T[1,1] = np.cos(theta)*np.cos(alpha)
		T[1,2] = -1*np.cos(theta)*np.sin(alpha)
		T[1,3] = np.sin(theta)*a
		
		#third row
		T[2,1] = np.sin(alpha)
		T[2,2] = np.cos(alpha)
		T[2,3] = d
		
		#fourth row
		T[3,3] = 1
		
		return T

	def get_pose(self, angles):
		
		pose = np.identity(4)

		for i in range(len(self.dh_params)):
			Ti = self.dh_to_transform(angles[i], self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], self.dh_params[i][3])
			pose = np.matmul(pose,Ti)

		return pose

	def old_scipy_version(self):
		
		version = [int(n) for n in scipy.__version__.split(".")]
		
		if version[0] < 1 or (version[0]==1 and version[1] < 4):
			return True
		else:
			return False

	def pose_from_topic(self, msg):

		joint_angles = msg.position
		pose_matrix = self.get_pose(joint_angles)

		self.curr_pose.position.x = pose_matrix[0,3]
		self.curr_pose.position.y = pose_matrix[1,3]
		self.curr_pose.position.z = pose_matrix[2,3]

		if self.old_scipy_version:
			r = R.from_dcm(pose_matrix[0:3,0:3])
		else:
			r = R.from_matrix(pose_matrix[0:3,0:3])

		quat = r.as_quat()

		self.curr_pose.orientation.x = quat[0]
		self.curr_pose.orientation.y = quat[1]
		self.curr_pose.orientation.z = quat[2]
		self.curr_pose.orientation.w = quat[3]

		
if __name__ == '__main__':

    rospy.init_node('pose_publisher')

    pose_updater = PosePublisher()

    joint_subscriber = rospy.Subscriber('joint_states', JointState, pose_updater.pose_from_topic)

    pose_publisher = rospy.Publisher('ur5e_pose', Pose, queue_size=10)

    while not rospy.is_shutdown():

        pose_publisher.publish(pose_updater.curr_pose)

