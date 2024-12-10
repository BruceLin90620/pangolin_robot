# quadruped_robot_4_DOF
Developed a biomimetic triceratops robot using ROS2 and computer vision, integrating Realsense D435 camera with Nvidia Isaac ROS VSLAM and AprilTag SLAM for indoor localization, while implementing Nav2 for autonomous navigation capabilities.

<img src="./images/portfolio_triceratops.jpg" width="640px">

## **Requirements**

- Python 3.10
- [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Orin Nano environment setup
```
$ mkdir pangolin_ws && cd pangolin_ws
```
```
$ git clone https://github.com/BruceLin90620/ros2-pangolin-robot.git
```
```
$ git checkout dev
```

- build:
```
$ pip3 install setuptools==58.2.0
```
```
$ colcon build --symlink-install
```

- cd /home/user: 
```
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
- cd /DynamixlSDK/python: 
```
$ sudo python setup.py install
```
- Isaac ROS install:
```
$ VSlAM ...
```
### Pangolin Start
```
$ ./start.sh
```
```
$ ros2 launch pangolin_bringup pangolin_bringup.launch.py
```

### VSLAM Test


### Pangolin launch
```
$ cd pangolin_ws/
```
```
$ source install/setup.bash
```
```
$ ros2 launch pangolin_control drive_controller.launch.py
```
### Pangolin keybord Control
```
$ cd pangolin_ws
```
```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```