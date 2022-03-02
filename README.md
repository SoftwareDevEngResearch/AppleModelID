# AppleModelID

The code in this repository performs live parameter estimation for an apple picking model using ROS. This code was developed for use in the Intelligent Machines and Materials Lab (IMML) at Oregon State University, and contains ROS nodes that can be used for systems using the UR5e and IMML AppleHand.

## Setup

### Computer

To use the code in this repository, you should use a computer running Ubuntu and the appropriate ROS distribution for your Ubuntu version. Instillation instructions for ROS can be found here: [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation). 

This repository should be cloned to your ROS workspace folder under the src subfolder. For instructions on setting up a ROS workspace, you can go to [http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Additionally, this package requires scipy and numpy todo: put version info and links.

### Robot

A force/torque sensor at the wrist of a robotic manipulator is assumed for the code in this repository. The robot must also have knowledge of its configuration. Any robotic manipulator and gripper may be used so long as these requirements are met.

## How to Use

### Required Topics

Before running any of the code here, you will need ROS to be publishing the robot's wrist pose as a Pose message and the force/torque data as a WrenchStamped message. Below you will find details on how to achieve this for the UR5e.

### How to Run the Code

You can use roslaunch to run any of the included launchfiles. Which launchfile to run depends on your goal:

To run the regression constantly, run 

```
roslaunch apple_modid_ros basic_regression.launch
```

To run the regression based on a trigger topic, run


```
roslaunch apple_modid_ros regression.launch regression_trigger:=trigger_topic
```

replacing trigger_topic for the name of your trigger topic. To use the trigger topic for the apple hand, once it is being published, replace trigger_topic with run_regression.

To publish a trigger topic based on slip detection for the apple hand, run

```
roslaunch apple_modid_ros grasp_adjustment_detection.launch
```

todo:correct this

To run only the grasp adjustment detection for the applehand, run

```
roslaunch apple_modid_ros grasp_adjustment_detection.launch
```

or, if a webcam is setup, you can run

```
roslaunch apple_modid_ros visualize_grasp_adjustment.launch
```

which includes visualization of the grasp detection process alongside the detection itself. 

If you want to use a robot other than the UR5e, you will need to specify 

```
use_ur5e:=false pose_topic:=your_topic_name
```

after the roslaunch command. Replace your_topic_name with the name of the topic which contains the pose information for the robot's wrist.

### Defining ROSParams

Many launch files require a .yaml file as input. In these files you will need to define the following fields:

pose_topic: this is the name of the ROS topic that contains the pose information for the wrist of the robot. 
wrench_topic: this is the name of the ROS topic that contains the force/torque data from the wrist.

### using applehand

### using the UR5e

### creating a user-defined trigger topic


