# AppleModelID

The code in this repository performs live parameter estimation for an apple picking model using ROS. This code was developed for use in the Intelligent Machines and Materials Lab (IMML) at Oregon State University, and contains ROS nodes that can be used for systems using the UR5e and IMML AppleHand.

## Setup

### Computer

To use the code in this repository, you should use a computer running Ubuntu and the appropriate ROS distribution for your Ubuntu version. Instillation instructions for ROS can be found here: [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation). 

This repository should be cloned to your ROS workspace folder under the src subfolder. For instructions on setting up a ROS workspace, you can go to [http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Additionally, this package requires scipy and numpy. Any version should work, but the programs were designed using scipy version 1.2.3 and numpy version 1.16.6.

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

Similarly, if the topic containing the wrenches on the robot is named anything other than "wrench", you will need to remap the wrench topic using:

```
wrench:=wrench_topic_name
```

as an additional argument, where wrench_topic_name is the name of your wrench topic.


### Using the IMML AppleHand

To process topics from the IMML Applehand, you will need to have the applehand ROS package installed. The package, as well as instructions for how to use the hand, are located at [https://github.com/cravetzm/Apple-Hand](https://github.com/cravetzm/Apple-Hand).

### Using the UR5e

To use the UR5e with this software, you will need the appropriate ROS drivers, available (with instructions) at [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). Running the following will publish the needed topics, provided that the robot is on and has an established connection with the computer:

```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=IP_ADDRESS
```

where IP address is the address where the robot can be reached.

Unlike the AppleHand, no software installation is required to play back topics from the UR5e that were recorded in a rosbag file. Only built-in rostopics are published by these drivers.


### Running the Examples

Data from an example pick is included in the form of two rosbag files. all_topics_example.bag contains pose, wrench, and applehand topics. After running any of the provided launch files (except visualize_grasp_adjustment.launch, because the webcam topics werer too large for github), playback the data using:

```
rosbag play all_topics_example.bag
```

to play back the data. The [rostopic command line tool](http://wiki.ros.org/rostopic) can be used to check the parameter estimates at any timestep, or the [rosbag command line tool](http://wiki.ros.org/rosbag) can be used to record this information for the whole file.

The second included rosbag file, no_hand_example.bag contains the same data but with the applehand topics removed. This allows for testing basic_regression.launch or regression.launch without installing the applehand package. Like with the other bagfile, simply run one of these two launch files and then use "rosbag play" in the command line.
