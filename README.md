# TeleSign: An Intuitive and Flexible Gesture-Based Teleoperation Framework

---

## Introduction

This repository contains the necessary instructions and dependencies to run TeleSign v1.0.

## Requisites

- Python 3.10 (preferred 3.10.12)
- MediaPipe 0.10.15
- OpenCV 4.6.0.66
- Nep 0.5.4.8 
- ROS2 Humble
- IsaacSim 4.2.0
- Webcam / Thinklet
- scrcpy

## Tutorial
- Network

For faster processing, the computer running TeleSign and the computer running IsaacSim must be connected to the same Wi-Fi. It is possible to run TeleSign in the same computer as IsaacSim, but it is recommended to use different computers for TeleSign and IsaacSim. 

To use Thinklet, the device must be used in the same Wi-Fi connection. As well, it is recommended to connect the TeleSign computer to the UR5e robot through Ethernet connection.


- Thinklet

1. Connect Thinklet device to computer using through USB

2. Open terminal and use "scrcpy"

3. Connect device to same Wi-Fi network.

![Images/WiFiThinklet.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/WiFiThinklet.png)

4. In TeleSign computer, open new terminal and run "nep master"

5. Check computer IP (ifconfig or nep ip)

6. In Thinklet device, go to AndroidCamera, put the TeleSign computer IP and click connect button.

![Images/AndroidCamera.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/AndroidCamera.png)

7. Disconnect Thinklet device from computer.

- Isaac Sim

1. Download TeleSign Workspace and Extension

2. In the TeleSign Extension folder, go to RmpFlow_Example_python and open scenario.py

3. In variable "path_to_usd", write directory to "UR5e_TeleSign.usd" file and save.

![Images/PathRobotUSD.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/PathRobotUSD.png)

4. Open Omniverse Launcher

5. Launch Isaac Sim

6. In the Isaac Sim App Selector click on `Open in Terminal` and source ROS2 and the TeleSign workspace by writing the following commands

![Images/OpenTerminal.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/OpenTerminal.png)

```bash
#Source global ROS2
source /opt/ros/humble/setup.bash

#Access the TeleSign workspace
cd ~/Documents/TeleSign/sign_lang #This may change depending on files

#Source the workspace
source install/setup.bash

#Return to Isaac Sim directory
cd /home/aist/.local/share/ov/pkg/isaac-sim-4.2.0

#Initiate Isaac Sim
./isaac-sim.sh


```

7. After launching Isaac Sim, go to `Window` -> `Extensions`

![Images/Extensions.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/Extensions.png)

8. Click on the three bars icon and select `Settings`

![Images/ExtensionSettings.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/ExtensionSettings.png)

9. In `Extension Search Paths` click on the plus sign to add a new path, after that write the directory where the Thinklet Extension is located

For example: `/home/aist/Documents/TeleSign/Extension_TeleSign`

![Images/ExtensionSearchPath.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/ExtensionSearchPath.png)

10. Click on `THIRD PARTY` -> `User` and enable the extension

![Images/TeleSignRMPFlow.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/TeleSignRMPFlow.png)

11. A new option called `RMPFlow Example` will appear on the top menu bar click on it and a new menu will appear. 

![Images/RMPFlow.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/RMPFlow.png)

12. On `World Controls` click on `LOAD`

13. Once the the scenario loads up, go to `Run Scenario` and click on `RUN`

![Images/RUNMRTwin.png](https://github.com/enriquecoronado-art/TeleSign/blob/main/Images/RUNMRTwin.png)

- TeleSign final steps

1. Run the following command in the TeleSign computer.

```bash

ros2 launch sign_agent telesign.launch.py device_used:=Thinklet #or webcam

```

2. Run the following commands in the IsaacSim computer.

```bash

#Open RViz and connect with Real UR5e Robot in first terminal
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=your-robot-ip launch_rviz:=true

#Enable forward position controller in another terminal
ros2 control switch_controllers --activate forward_position_controller --deactivate scaled_joint_trajectory_controller

#Go to the workspace in another terminal
cd ~/Documents/TeleSign/sign_lang

#Source the workspace
source install/setup.bash

#Activate gripper for UR5e 
ros2 run ur5e_control activate_gripper

```

3. In UR5e Teach-Pendant, push Play button.

4. In another terminal on IsaacSim computer, run these commands

```bash

#Go to the workspace in another terminal
cd ~/Documents/TeleSign/sign_lang

#Source the workspace
source install/setup.bash

#Run publisher for Real UR5e
ros2 run ur5e_control joint_control

```