# ROS_DMS
## Driver Monitoring System

This repository is to monitor a car's driver during the drive. It can detect the driver's `Eye`, `Yawn` and `Head Position`. Also, it can find the eye close and open threshold automatically.

## How to install
### System Requirements
It has been tested uisng:
- ROS Melodic/Noetic
- Ubuntu 18.04/20.04
- Pyhton3.6 +




# ROS_DMS: Driver Monitoring System

ROS_DMS is a Driver Monitoring System (DMS) for real-time detection of driver drowsiness and attention using computer vision and ROS. It monitors the driver's eyes, yawning, and head position, and automatically calibrates eye open/close thresholds.

## Features
- Eye state detection (open/close)
- Yawn detection
- Head position estimation
- Automatic threshold calibration
- Real-time alerts (audio)

## System Requirements
- ROS Melodic or Noetic
- Ubuntu 18.04 or 20.04
- Python 3.6+

## Installation

### 1. Install Python dependencies
```bash
pip3 install -r requirements.txt
```
Or manually:
```bash
pip3 install imutils rospy opencv-python numpy pygame dlib scipy
```

### 2. Install ROS catkin tools (if not installed)
```bash
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### 3. Build the ROS workspace
```bash
cd <catkin_ws_path>
catkin build
source <catkin_ws_path>/devel/setup.bash
```

## Usage

Launch the DMS node:
```bash
roslaunch dms_conigital dms.launch
```
This will start the camera and run the driver monitoring system. Alerts will be played using the media files in the `media/` folder.

## Folder Structure
- `scripts/` : Python scripts for camera, detection, and pose estimation
- `media/`   : Audio files for alerts
- `model/`   : Pre-trained models and configuration
- `msg/`     : Custom ROS message definitions
- `launch/`  : Launch files for ROS

## Contributing
Pull requests and issues are welcome! Please open an issue for bug reports or feature requests.

## Author
- **Mohammad Bamorovat**
- Email: m.bamorovvat@gmail.com
- Website: https://www.bamorovat.com

## License
This project is licensed under the MIT License.

