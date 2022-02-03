#!/usr/bin/env python

import rospy
from applehand.msg import ImuNaive
from std_msgs.msg import Bool

class AdjustmentDetector:

    def __init__(self):

        self.adjusting = False
	self.f1_in_bounds = True
        self.f2_in_bounds = True
        self.f3_in_bounds = True

        self.thresh = 10 #tuned parameter for deg/s during grasp adjustment

    #This is called by the callback function for the IMU topic subscribers. If a single IMU
    #changes between being within movement thresholds to without these thresholds, this 
    #will return true
    def detect_imu_changes(self, msg):
        if abs(msg.angular_velocity.x) > self.thresh:
            return False
        else:
            return True

    #This function is run constantly by the node in order to update the grasp adjustment state
    def detect_adjustment(self):
        self.adjusting = not (self.f1_in_bounds and self.f2_in_bounds and self.f3_in_bounds)

    #The actual callback functions for the IMU subscribers
    def update_f1(self, msg):
        self.f1_in_bounds = self.detect_imu_changes(msg)
    
    def update_f2(self, msg):
        self.f2_in_bounds = self.detect_imu_changes(msg)

    def update_f3(self, msg):
        self.f3_in_bounds = self.detect_imu_changes(msg)

if __name__ == '__main__':

    rospy.init_node('grasp_adjustment_detector')

    detector = AdjustmentDetector()

    f1_subscriber = rospy.Subscriber('applehand/finger1/imu', ImuNaive, detector.update_f1)
    f2_subscriber = rospy.Subscriber('applehand/finger2/imu', ImuNaive, detector.update_f2)
    f3_subscriber = rospy.Subscriber('applehand/finger3/imu', ImuNaive, detector.update_f3)

    adjustment_publisher = rospy.Publisher('grasp_adjusting', Bool, queue_size=10)

    while not rospy.is_shutdown():

        detector.detect_adjustment()
        adjustment_publisher.publish(detector.adjusting)
