# ROS Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## About:
This repository is part of the course for ENPM808x - Software Development for Robotics. The original tutorial from ROS documentation has been modified to meet Google's C++ Style guide. 

## Dependencies and Assumptions:
1. This software package was tested on ROS Melodic for Ubuntu 18.04.
2. The source code is built with C++ 11 features and compiled with g++ compiler.
3. The project is built with catkin, which is a modified version of CMake. 

## Create Catkin workspace:
From any desired directory
```
mkdir ws/src
cd ws
```

## Cloning this repository:
```
git clone --recursive https://github.com/aswathselvam/beginner_tutorials.git src/beginner_tutorials
```

## Build source:
Build the source files using:
```
catkin_make
```

## Run the executables
### Publisher:
```
rosrun beginner_tutorials talker
```

### Subscriber:
The output can be seen by running:
```
rosrun beginner_tutorials listener
```
or
```
rostopic echo chatter
```

### ROS Service:
To set custom message to Talker:
```
rosservice call /service "input-your-message-here"
```

To kill listener node:
```
rosservice call /Kill
```

### ROS Launch:
Launch both talker and subscriber node:
```
roslaunch beginner_tutorials pubsub.launch message:="your-custom-message"
```

## View ROS TF Frames
```
rosrun tf tf_echo /world /talk
rosrun rqt_tf_tree rqt_tf_tree
```
Save TF frame to pdf:
```
rosrun tf view_frames
```

## Run ROS Test
Make tests
```
catkin_make tests
catkin_make test
```
Run the Test case Launch file:
```
rostest beginner_tutorials pubsub_test.launch 
```

## ROS Bag
To record talker node's messages with ROS Bag.
```
roslaunch beginner_tutorials pubsub.launch rosbag_record:=true
```

To play the bag file:
```
rosbag play "path-to-bag-file"
```

Run the listener node to inspect the contents of the bag-file with topic name "/chatter". Other 
topics that maybe recorded can be viewed with "rostopic list"

## Code Formatting check 
### Cpplint:
```
cd src/beginner_tutorials
cpplint $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/") > results/cpplintoutput.txt
```
### Cppcheck:
```
cppcheck --enable=all --std=c++11 --language=c++ -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/") > results/cppcheckoutput.txt
```

