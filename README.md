## ROS Beginner Tutorials

### About:
This repository is part of the course for ENPM808x - Software Development for Robotics. The original tutorial from ROS documentation has been modified to meet Google's C++ Style guide. 

### Dependencies and Assumptions:
1. This software package was tested on ROS Melodic for Ubuntu 18.04.
2. The source code is built with C++ 11 features and compiled with g++ compiler.
3. The project is built with catkin, which is a modified version of CMake. 

### Create Catkin workspace:
From any desired directory
```
mkdir ws/src
cd ws
```

### Cloning this repository:
```
git clone --recursive https://github.com/aswathselvam/beginner_tutorials.git src/beginner_tutorials
```

### Build source:
Build the source files using:
```
catkin_make
```

### Run the executable
```
rosrun beginner_tutorials talker
```

The output can be seen by running:
```
rosrun beginner_tutorials listener
```
or
```
rostopic echo chatter
```

### Code Formatting check 
#### Cpplint:
```
cd src/beginner_tutorials
cpplint $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/") > results/cpplintoutput.txt
```
#### Cppcheck:
```
cppcheck --enable=all --std=c++11 --language=c++ -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/") > results/cppcheckoutput.txt
```

