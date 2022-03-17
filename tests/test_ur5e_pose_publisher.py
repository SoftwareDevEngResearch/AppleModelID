#!/usr/bin/env python

#We're going to make some fake ROS messages to test, so I will import some ROS stuff
import rospy 

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

#import the class to test
import sys
sys.path.append("/home/miranda/catkin_ws/src/AppleModelID/apple_modid_ros/src")

from ur5e_pose_publisher import PosePublisher

#other useful packages
import numpy as np

#--- Helper Functions  ----------------------------------------------------------------------------

#just to make sure numpy.allclose() works 
def pose_as_array(msg):
	
	pose_array = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
	return pose_array
	

#--- Tests ----------------------------------------------------------------------------------------

def test_zero_pose():

	correct_pose = np.array([-0.8172, -0.2329, 0.0628, 0.7071068, 0, 0, 0.7071068])

	temp_obj = PosePublisher()

	to_send = JointState()
	to_send.position = [0, 0, 0, 0, 0, 0]

	temp_obj.pose_from_topic(to_send)
	new_pose = pose_as_array(temp_obj.curr_pose)
	
	assert(np.allclose(new_pose, correct_pose, atol=1e-3))

def test_arm_up():

	correct_pose = np.array([-0.0997, -0.2329, 0.9797, -0.5, -0.5, 0.5, -0.5])
	alternative_correct_pose = np.array([-0.0997, -0.2329, 0.9797, 0.5, 0.5, -0.5, 0.5]) #equivalent quaternion

	temp_obj = PosePublisher()

	to_send = JointState()
	to_send.position = [0, 4.7124, 0, 0, 0, 0]

	temp_obj.pose_from_topic(to_send)
	new_pose = pose_as_array(temp_obj.curr_pose)
	
	assert(np.allclose(new_pose, correct_pose, atol=1e-3) or np.allclose(new_pose, alternative_correct_pose, atol=1e-3))

def test_wrist_above_base():

	correct_pose = np.array([-0.09369, -0.23290, 0.81085, -0.2706, -0.6533, 0.6533, -0.2706])

	temp_obj = PosePublisher()

	to_send = JointState()
	to_send.position = [0, 5.4978, 4.7124, 0, 0, 0]

	temp_obj.pose_from_topic(to_send)
	new_pose = pose_as_array(temp_obj.curr_pose)
	
	assert(np.allclose(new_pose, correct_pose, atol=1e-3))

#this is where we usually start the trials 
def test_experiment_pose():

	correct_pose = np.array([0.2742, -0.5742, 0.6238, 0.7341, 0.0562, -0.5235, -0.4288])

	temp_obj = PosePublisher()

	to_send = JointState()
	to_send.position = [5.4105, 3.4907, 0.2618, 5.7596, 4.9742, 0.7854]

	temp_obj.pose_from_topic(to_send)
	new_pose = pose_as_array(temp_obj.curr_pose)
	
	assert(np.allclose(new_pose, correct_pose, atol=1e-3))

def test_all_45():

	correct_pose = np.array([0.0166, -0.2715, -0.5095, -0.5, 0.7071, -0.3536, 0.3536])

	temp_obj = PosePublisher()

	to_send = JointState()
	to_send.position = [0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854]

	temp_obj.pose_from_topic(to_send)
	new_pose = pose_as_array(temp_obj.curr_pose)
	
	assert(np.allclose(new_pose, correct_pose, atol=1e-3))

def test_all_90():

	correct_pose = np.array([0.1333, 0.2925, -0.1629, 0, 0, 0.7071, 0.7071])

	temp_obj = PosePublisher()

	to_send = JointState()
	to_send.position = [1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 1.5708]

	temp_obj.pose_from_topic(to_send)
	new_pose = pose_as_array(temp_obj.curr_pose)
	
	assert(np.allclose(new_pose, correct_pose, atol=1e-3))
