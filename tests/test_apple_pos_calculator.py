#!/usr/bin/env python

#We're going to make some fake ROS messages to test, so I will import some ROS stuff
import rospy 

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64

#import the class to test
import sys
sys.path.append("/home/miranda/catkin_ws/src/AppleModelID/apple_modid_ros/src")

from apple_pos_calculator import ApplePositionCalculator

#other useful packages

import numpy as np
import random


#--- Tests ----------------------------------------------------------------------------------------

#This test checks that if l is zero, the apple position updates to the position of the wrist

def test_impossible_apple():
	
	temp_calculator = ApplePositionCalculator()

	#create a random pose
	to_test = Pose()
	to_test.position.x = random.random()
	to_test.position.y = random.random()
	to_test.position.z = random.random()
	to_test.orientation.x = random.random()
	to_test.orientation.y = random.random()
	to_test.orientation.z = random.random()
	to_test.orientation.w = random.random()


	#make sure the position matches this pose after update
	temp_calculator.calculate_position(to_test)
	assert(temp_calculator.position == to_test.position)


#These tests show that the apple is along the z axis of the hand for z axes that are aligned with x, y, or z in the world frame
#wrist is at the origin for this

def test_x_axis():
	
	temp_calculator = ApplePositionCalculator()

	dist = Float64()
	dist.data = random.random() #random l
	temp_calculator.update_l(dist)

	#x axis
	to_test = Pose()

	to_test.orientation.x = 0.5
	to_test.orientation.y = 0.5
	to_test.orientation.z = 0.5
	to_test.orientation.w = 0.5

	correct_position = Point()
	correct_position.x = dist.data
	
	temp_calculator.calculate_position(to_test)

	assert(temp_calculator.position == correct_position)

def test_y_axis():
	
	temp_calculator = ApplePositionCalculator()

	dist = Float64
	dist.data = random.random() #random l
	temp_calculator.update_l(dist)

	#x axis
	to_test = Pose()

	to_test.orientation.x = 0.5
	to_test.orientation.y = 0.5
	to_test.orientation.z = 0.5
	to_test.orientation.w = -0.5

	correct_position = Point()
	correct_position.y = dist.data
	
	temp_calculator.calculate_position(to_test)

	assert(temp_calculator.position == correct_position)

def test_z_axis():
	
	temp_calculator = ApplePositionCalculator()

	dist = Float64
	dist.data = random.random() #random l
	temp_calculator.update_l(dist)

	#x axis
	to_test = Pose()

	to_test.orientation.x = 0
	to_test.orientation.y = 0
	to_test.orientation.z = 0
	to_test.orientation.w = 1

	correct_position = Point()
	correct_position.z = dist.data
	
	temp_calculator.calculate_position(to_test)

	assert(temp_calculator.position == correct_position)

