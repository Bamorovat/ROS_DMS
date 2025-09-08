# ROS_DMS: Driver Monitoring System

![ROS](https://img.shields.io/badge/ROS-Melodic%20%7C%20Noetic-blue.svg)
![Python](https://img.shields.io/badge/Python-3.6%2B-green.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-red.svg)
![Build](https://img.shields.io/badge/Build-Catkin-orange.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2018.04%20%7C%2020.04-lightgrey.svg)

🔗 **Project Page:** [https://bamorovat.com/projects/dms-av.html](https://bamorovat.com/projects/dms-av.html)

ROS_DMS is a Driver Monitoring System (DMS) for real-time detection of driver drowsiness and attention using computer vision and ROS. It monitors the driver's eyes, yawning, and head position, and automatically calibrates eye open/close thresholds.

## Features
- Eye state detection (open/close)
- Yawn detection
- Head position estimation
- Automatic threshold calibration
- Real-time alerts (audio)

## System Requirements

> [!IMPORTANT]
> Make sure you have a compatible ROS distribution installed before proceeding.

- ROS Melodic or Noetic
- Ubuntu 18.04 or 20.04
- Python 3.6+
- USB Camera (compatible with OpenCV)

## Installation

> [!TIP]
> It's recommended to create a new catkin workspace for this project.

### 1. Clone the repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/Bamorovat/ROS_DMS.git
```

### 2. Install Python dependencies
```bash
cd ~/catkin_ws/src/ROS_DMS
pip3 install -r requirements.txt
```

> [!NOTE]
> If you don't have a requirements.txt file, install manually:
> ```bash
> pip3 install imutils rospy opencv-python numpy pygame dlib scipy
> ```

### 3. Install ROS catkin tools (if not installed)
```bash
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### 4. Build the ROS workspace
```bash
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
```

> [!IMPORTANT]
> Make sure to source your workspace in every new terminal session, or add it to your `~/.bashrc`:
> ```bash
> echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
> ```

## Usage

> [!WARNING]
> Ensure your camera is connected and accessible before launching the system.

### Launch the DMS system
```bash
roslaunch dms_conigital dms.launch
```

> [!TIP]
> You can customize camera parameters by editing the launch file or passing arguments:
> ```bash
> roslaunch dms_conigital dms.launch camera_index:=1
> ```

This will start the camera and run the driver monitoring system. Alerts will be played using the media files in the `media/` folder.

### Monitoring Topics
You can monitor the system status using:
```bash
# View camera feed
rostopic echo /center_cam

# View drowsiness detection results
rostopic echo /drowsiness_status
```

## 📁 Folder Structure
```
ROS_DMS/
├── 📂 scripts/          # Python scripts for camera, detection, and pose estimation
├── 📂 media/            # Audio files for alerts
├── 📂 model/            # Pre-trained models and configuration
├── 📂 msg/              # Custom ROS message definitions
├── 📂 launch/           # Launch files for ROS
├── 📄 CMakeLists.txt    # Build configuration
├── 📄 package.xml       # Package dependencies
├── 📄 requirements.txt  # Python dependencies
└── 📄 README.md         # This file
```

## Contributing

> [!NOTE]
> We welcome contributions from the community!

Pull requests and issues are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

> [!TIP]
> Before submitting, please ensure your code follows the existing style and includes appropriate tests.

## Author
- **Mohammad Bamorovat**
- Email: m.bamorovvat@gmail.com
- Website: https://www.bamorovat.com

## License
This project is licensed under the MIT License.

---

> [!NOTE]
> If you find this project useful, please consider giving it a ⭐ star on GitHub!

