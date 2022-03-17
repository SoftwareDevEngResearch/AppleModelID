#!/usr/bin/env python

#We're going to make some fake ROS messages to test, so I will import some ROS stuff
import rospy 

from geometry_msgs.msg import Point, WrenchStamped, Vector3
from std_msgs.msg import Bool

#import the class to test
import sys
sys.path.append("/home/miranda/catkin_ws/src/AppleModelID/apple_modid_ros/src")

from find_params import AppleRegression

#other useful packages

import numpy as np
import random

#--- Tests ----------------------------------------------------------------------------------------

# Check that regression turns on and off as expected

def test_regression_on():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	
	temp_obj.check_regression(turn_on)

	assert(temp_obj.running==True)


def test_regression_off():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	turn_off = Bool
	turn_off.data = False
	
	temp_obj.check_regression(turn_on)
	temp_obj.check_regression(turn_off)

	assert(temp_obj.running==False)

def test_regression_stays_on():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	
	for i in range(4):
		temp_obj.check_regression(turn_on)

	assert(temp_obj.running==True)

def test_regression_on_until_off():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	turn_off = Bool
	turn_off.data = False
	
	for i in range(5):
		temp_obj.check_regression(turn_on)

	temp_obj.check_regression(turn_off)

	assert(temp_obj.running==False)

# Check that the lists cap off at the correct size
def test_regression_sizing():

	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	
	temp_obj.check_regression(turn_on)

	for i in range(200):
		position_msg = Point()
		
		position_msg.x = random.random()
		position_msg.y = random.random()
		position_msg.z = random.random()

		force_msg = WrenchStamped()
		
		force_msg.wrench.force.x = random.random()
		force_msg.wrench.force.y = random.random()
		force_msg.wrench.force.z = random.random()

		temp_obj.update_position(position_msg)
		temp_obj.update_force(force_msg)
	
	assert(len(temp_obj.forces['x']) == temp_obj.chunk_size and len(temp_obj.positions['x']) == temp_obj.chunk_size)
		

# Check that regression is adequate for single dimensions
# true location is 1,1,1 for all examples

def test_x_regression():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	
	temp_obj.check_regression(turn_on)

	x_positions = np.linspace(0,1,101)
	true_k = 600 #the spring stiffness for the proxy orchard is 632
	x_forces = true_k*(1-x_positions)

	position_noise = np.random.normal(0,0.01,101) #1 cm is way more positional noise than the UR5e sensors
	force_noise = np.random.normal(0,0.1,101) #0.1 N error is also very conservative	

	x_positions = x_positions + position_noise
	x_forces = x_forces + force_noise

	for i in range(101):

		position_msg = Point()
		
		position_msg.x = x_positions[i]

		#prevent regression from being degenerate due to arrays of zeros
		position_msg.y = 1
		position_msg.z = 1	
	
		force_msg = WrenchStamped()
		
		force_msg.wrench.force.x = x_forces[i]

		temp_obj.update_position(position_msg)
		temp_obj.update_force(force_msg)

	temp_obj.run_regression()
	
	error = temp_obj.param_est['x'] - 1

	assert(error < 0.01)
	
def test_y_regression():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	
	temp_obj.check_regression(turn_on)

	y_positions = np.linspace(0,1,101)
	true_k = 600 #the spring stiffness for the proxy orchard is 632
	y_forces = true_k*(1-y_positions)

	position_noise = np.random.normal(0,0.01,101) #1 cm is way more positional noise than the UR5e sensors
	force_noise = np.random.normal(0,0.1,101) #0.1 N error is also very conservative	

	y_positions = y_positions + position_noise
	y_forces = y_forces + force_noise

	for i in range(101):

		position_msg = Point()
		
		position_msg.y = y_positions[i]

		#prevent regression from being degenerate due to arrays of zeros
		position_msg.x = 1
		position_msg.z = 1	
	
		force_msg = WrenchStamped()
		
		force_msg.wrench.force.y = y_forces[i]

		temp_obj.update_position(position_msg)
		temp_obj.update_force(force_msg)

	temp_obj.run_regression()
	
	error = temp_obj.param_est['y'] - 1

	assert(error < 0.01)

def test_z_regression():
	
	temp_obj = AppleRegression()

	turn_on = Bool
	turn_on.data = True
	
	temp_obj.check_regression(turn_on)

	z_positions = np.linspace(0,1,101)
	true_k = 600 #the spring stiffness for the proxy orchard is 632
	z_forces = true_k*(1-z_positions)

	position_noise = np.random.normal(0,0.01,101) #1 cm is way more positional noise than the UR5e sensors
	force_noise = np.random.normal(0,0.1,101) #0.1 N error is also very conservative	

	z_positions = z_positions + position_noise
	z_forces = z_forces + force_noise

	for i in range(101):

		position_msg = Point()
		
		position_msg.z = z_positions[i]

		#prevent regression from being degenerate due to arrays of zeros
		position_msg.x = 1
		position_msg.y = 1	
	
		force_msg = WrenchStamped()
		
		force_msg.wrench.force.z = z_forces[i]

		temp_obj.update_position(position_msg)
		temp_obj.update_force(force_msg)

	temp_obj.run_regression()
	
	error = temp_obj.param_est['z'] - 1

	assert(error < 0.01)










