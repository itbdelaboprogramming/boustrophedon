# Boustrophedon Path Planning

## Requirements
1. ROS Noetic
2. Ubuntu 20.04 LTS

## How to Install
```bash
sudo apt update -y
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-bringup
sudo apt install ros-noetic-turtlebot3-gazebo 
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3-navigation
sudo apt install ros-noetic-turtlebot3-simulations
sudo apt install ros-noetic-turtlebot3-slam
cd ~/catkin_ws/src
git clone https://github.com/itbdelaboprogramming/boustrophedon.git
cd ~/catkin_ws
catkin_make
```

## How to Run
```bash
cd ~/catkin_ws
source devel/setup.bash

# run simulator
roslaunch boustrophedon boustrophedon.launch

#run boustrophedon path
rosrun boustrophedon boustrophedon_node.py
```