# rp_project
RP project for Sapienza

## Usage



I ran these commands to setup ROS1 neotic on my ubuntu 22. This should be done after copying my code to any folder where we want to build the workspace:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-noetic-desktop-full

catkin_init_workspace

catkin init

catkin build

source devel/setup.bash 

catkin build

source devel/setup.bash 

roslaunch xy_simulator navigation.launch map_file:=cappero_map.yaml
```
