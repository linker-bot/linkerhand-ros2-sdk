# ROS 2 SDK Introduction

## 1. **Overview**

LinkerHand Dexterous Hand ROS SDK, developed by CHIUS INC, is the driving software for a series of LinkerHand dexterous hands, including models such as L7, O7, L10, and O10, and provides functional example source code. It is applicable for both real robots and simulators.&#x20;

Compatibility: Ubuntu 20.04 + ROS 2 Foxy + Python ≥3.8.20

## 2. Install

Ensure the system environment meets the following requirements:

Ubuntu 22.04、ROS 2 Humble、Python 3.10 or later

Download

```bash
  $ mkdir -p linker_hand_ros2_sdk/src
  $ cd linker_hand_ros2_sdk/src
  $ git clone https://github.com/linkerbotai/linker_hand_ros2_sdk.git
```

* Compile

```bash
  $ cd linker_hand_ros2_sdk/src/
  $ sudo apt install python3-can
  $ pip install -r requirements.txt
```

## 3. Usage

**Before use, modify the setting.yaml configuration file according to your requirements.**

**Update line 28 in load\_write\_yaml.py by setting yaml\_path ="" to the absolute path of the config directory**

* Launch SDK    

  * Insert the USB-to-CAN device interface of the LinkerHand dexterous hand into the Ubuntu device.

```bash
  # Enable CAN port
  $ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB-to-CAN device with blue light constantly on
  $ cd linker_hand_ros2_sdk/
  $ colcon build
  $ source ./install/setup.bash
  $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
```
* Launch the status waveform plot (for LinkerHand with pressure sensors) 
```bash 
 # Open a new terminal after launching the ROS 2 SDK 
 $ cd linker_hand_ros2_sdk/ 
 $ source ./install/setup.bash 
 $ ros2 run graphic_display graphic_display
```
## 4. Version Information
* release_2.1.4

  * 1、Support matrix pressure sensors
  * 2、with newly added support for L21

* release_1.0.3

  * 1、Compatible with L20/L25 Dexterous Hand Models

* release_1.0.2

  * 1、Compatible with L10/O10 Dexterous Hand Models 

  * 2、Supports GUI-Based Control for L10/O10 Dexterous Hands

  * 3、Added real-time pressure sensor waveform visualization for the LinkerHand, enabling dynamic monitoring of tactile feedback status.
  
* release_1.0.1

  * 1、Compatible with L7/O7 Dexterous Hand Models

  * 2、Supports GUI-Based Control for L7/O7 Dexterous Hands

## 5. Examples

**Before use, modify the setting.yaml configuration file according to your requirements.**

**Update line 28 in load\_write\_yaml.py by setting yaml\_path ="" to the absolute path of the config directory**

### 5.1 General

* 0001-gui\_control(Graphical User Interface Control)
  After launching the ROS 2 SDK

```bash
# Open a new terminal
$ cd linker_hand_ros2_sdk/
$ source ./install/setup.bash
$ ros2 run gui_control gui_control
```

### 5.2 L7

* 7001-action-group-show-ti(Finger Movement)

### 5.3 L10

* 10001-action-group-show-normal(Finger Movement)

