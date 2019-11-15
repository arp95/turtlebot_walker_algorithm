# Software Development for Robotics (ENPM808x) ROS Exercise

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)


## Overview of the project

Modified the Turtlebot simulation and implemented a simple walker algorithm like Roomba robot vacuum cleaner. The robot moves forward until it reaches an obstacle and then rotates in place until the way ahead is clear, then moves forward and repeats the above process.


## Dependencies

The following dependencies are required to run this package:

1. ROS kinetic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 16.04 For installing ROS (http://wiki.ros.org/kinetic/Installation)


## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/arp95/turtlebot_walker_algorithm
cd ..
catkin_make
```

Open the following terminals and run the following commands in them:

1. Terminal 1:
```
roscore
```

2. Terminal 2:
Passing record=true for recording of bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker_algorithm turtlebot_walker_algorithm.launch record:=true
```

Passing record=false for not recording the bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker_algorithm turtlebot_walker_algorithm.launch record:=false
```
