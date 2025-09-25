# This project has been moved to the new address, please visit [https://github.com/linker-bot/linkerhand-ros2-sdk](https://github.com/linker-bot/linkerhand-ros2-sdk)


------

# LinkerHand Dexterous Hand ROS2 SDK  

## Overview  
The LinkerHand Dexterous Hand ROS SDK is developed by LinkerHand (Beijing) Technology Co., Ltd. It provides driver software and functional example source code for LinkerHand dexterous hands such as the L7, O7, L10, and O10 models, supporting both physical devices and simulators.  
The LinkerHand ROS2 SDK currently supports Ubuntu 22.04, ROS Humble, and Python 3.10 or higher environments.  

## Installation  
&ensp;&ensp;Ensure the system environment meets the requirements: Ubuntu 20.04, ROS 2 Foxy, and Python 3.8.20 or higher.  
- **Download**  

```bash  
  $ mkdir -p linker_hand_ros2_sdk/src  
  $ cd linker_hand_ros2_sdk/src  
  $ https://github.com/linker-bot/linkerhand-ros2-sdk.git
```  

- **Build**  

```bash  
  $ sudo apt install python3-can  
  $ cd linker_hand_ros2_sdk/src/  
  $ pip install -r requirements.txt  
```  

## Usage for Ubuntu  
&ensp;&ensp; __Before use, modify the [setting.yaml](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml) configuration file according to your needs.__  
- Modify the password in `setting.yaml`. The default password is `"12345678"`, which corresponds to the Ubuntu system password for automatically enabling the CAN port in the SDK.  

&ensp;&ensp; __Before use, configure the [linker_hand.launch.py](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py) file according to the actual dexterous hand parameters.__  

- **Launch the SDK**  
  Connect the LinkerHand dexterous hand's USB-to-CAN device to the Ubuntu machine (supported models: L7, L10, L20, L21, L25).  

```bash  
  # Enable the CAN port  
  $ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000  # The USB-to-CAN device's blue LED will stay lit  
  $ cd linker_hand_ros2_sdk/  
  $ colcon build --symlink-install  
  $ source ./install/setup.bash  
  $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py  
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  Current SDK version: 2.1.4  
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set speed to [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]  
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set maximum torque to [200, 200, 200, 200, 200]  
```  

## Usage for WIN + ROS2  

&ensp;&ensp; __Before use, configure the [linker_hand.launch.py](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py) file according to the actual dexterous hand parameters.__  

- **Launch the SDK**  
  Connect the LinkerHand dexterous hand's USB-to-CAN device to the Windows machine (supported models: L7, L10, L20, L21, L25).  
  Note: Ensure the USB-to-CAN driver is installed before use.  

```bash  
  $ mkdir -p linker_hand_ros2_sdk/src  
  $ cd linker_hand_ros2_sdk/src  
  $ git clone https://github.com/linkerbotai/linker_hand_ros2_sdk.git  
  $ cd linker_hand_ros2_sdk/  
  $ set PYTHONUTF8=1  # Set environment variable to UTF-8 encoding  
  $ colcon build --symlink-install  
  $ call ./install/local_setup.bat  
  $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py  # Modify the CAN port name in the launch file first  
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  Current SDK version: 2.1.4  
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set speed to [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]  
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set maximum torque to [200, 200, 200, 200, 200]  
```  

- **Position-to-Finger Joint Mapping**  
```bash  
$ ros2 topic echo /cb_left_hand_control_cmd  
```  
```bash  
  header:  
    seq: 256  
    stamp:  
      secs: 1744343699  
      nsecs: 232647418  
    frame_id: ''  
  name: []  
  position: [155.0, 162.0, 176.0, 125.0, 255.0, 255.0, 180.0, 179.0, 181.0, 68.0]  
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
```  
  L7:  ["Thumb flexion", "Thumb abduction", "Index flexion", "Middle flexion", "Ring flexion", "Little flexion", "Thumb rotation"]  

  L10: ["Thumb base", "Thumb abduction", "Index base", "Middle base", "Ring base", "Little base", "Index abduction", "Ring abduction", "Little abduction", "Thumb rotation"]  

  L20: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction", "Thumb opposition", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]  

  L21: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]  

  L25: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb abduction", "Index abduction", "Middle abduction", "Ring abduction", "Little abduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle", "Index middle", "Middle middle", "Ring middle", "Little middle", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]  

## Version Updates
- > ### release_2.1.8
 - 1. Fix occasional frame collision issues

- > ### release_2.1.7  
 - 1. Fixed known issues.  
 - 2. Moved [Mujoco and PyBullet simulation](https://github.com/linkerbotai/linker_hand_sim) to a separate repository to reduce SDK size.  

- > ### release_2.1.6  
  - 1. Added support for dual CAN control of two dexterous hands.  
  - 2. Added Mujoco simulation.  
  - 3. Added PyBullet simulation.  

- > ### release_1.0.3  
  - 1. Added support for L20/L25 dexterous hands.  

- > ### release_1.0.2  
  - 1. Added support for L10/O10 dexterous hands.  
  - 2. Added GUI control for L10/O10 dexterous hands.  
  - 3. Added support for pressure sensor visualization in LinkerHand.  

- > ### release_1.0.1  
  - 1. Added support for L7/O7 dexterous hands.  
  - 2. Added GUI control for L7/O7 dexterous hands.  

## [Examples](examples/)  

&ensp;&ensp; __Before use, modify the [setting.yaml](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml) configuration file according to your needs.__  

## General  
- [gui_control (Graphical Interface Control)](图形界面控制)  
The graphical interface allows controlling individual joints of LinkerHand dexterous hands (L10, L20) via sliders. It also supports saving the current joint states by recording slider values and replaying actions using functional buttons.  

To control the LinkerHand dexterous hand via `gui_control`:  
The GUI requires the `linker_hand_sdk_ros` to be running, as it communicates with the hand via ROS topics.  

&ensp;&ensp; __Before use, configure the [gui_control.launch.py](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/gui_control/launch/gui_control.launch.py) file according to the actual dexterous hand parameters.__  

```bash  
# Open a new terminal  
$ cd linker_hand_ros2_sdk/  
$ source ./install/setup.bash  
$ ros2 launch gui_control gui_control.launch.py  
```  
The UI will open, allowing joint control via sliders.  

## Using GUI in WIN + ROS2  
&ensp;&ensp; __Before use, configure the [gui_control.launch.py](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/gui_control/launch/gui_control.launch.py) file according to the actual dexterous hand parameters.__  

```bash  
# Open a new terminal  
$ cd linker_hand_ros2_sdk/  
$ call ./install/setup.bash  
$ ros2 launch gui_control gui_control.launch.py  
```  

## L7  
- [7001-action-group-show-ti (Finger Motion)](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/examples/L7/gesture/action-group-show-ti.py)  

## L10  
- [10001-action-group-show-normal (Finger Motion)](https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/examples/L10/gesture/action-group-show-normal.py)  

## Topic Documentation  
[Linker Hand Topic Document](doc/Topic-Reference.md)  

## Mujoco and PyBullet Simulation  
 - [Mujoco and PyBullet Repository](https://github.com/linkerbotai/linker_hand_sim)
