# ROS_DMS
## Driver Monitoring System

This repository is to monitor a car's driver during the drive. It can detect the driver's `Eye`, `Yawn` and `Head Position`. Also, it can find the eye close and open threshold automatically.

## How to install
### System Requirements
It has been tested uisng:
- ROS Melodic/Noetic
- Ubuntu 18.04/20.04
- Pyhton3.6 +



### Install deps

```bash
  $ pip3 install imutils rospy opencv-python numpy pygame dlib scipy
```
 Install catkin tools if you do not have it:

```bash
  $ sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
  $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install python3-catkin-tools
```
### Build

```bash
  $ cd <catkin_ws_path>
  $ catkin build 
  $ source <catkin_ws_path>/devel/setup.bash
```

# How to run

Run:
```bash
$ roslaunch dms_conigital dms.launch
```
This will run the camera from USB and run the driver monitoring System.

