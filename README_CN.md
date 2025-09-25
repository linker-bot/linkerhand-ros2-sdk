# LinkerHand灵巧手ROS2 SDK

## 概述
LinkerHand灵巧手ROS SDK 是灵心巧手(北京)科技有限公司开发，用于O6、L6、L6P、L7、O7、L10、L21等LinkerHand灵巧手的驱动软件和功能示例源码。可用于真机与仿真器使用。
LinkerHandROS2 SDK当前支持Ubuntu22.04 ROS humble Python3.10 及以上环境

## 安装
&ensp;&ensp;确保当前系统环境为Ubuntu20.04 ROS 2 Foxy Python3.8.20 及以上
- 下载

```bash
  $ mkdir -p linker_hand_ros2_sdk/src
  $ cd linker_hand_ros2_sdk/src
  $ git clone https://github.com/linker-bot/linkerhand-ros2-sdk.git
```

- 编译

```bash
  $ sudo apt install python3-can
  $ cd linker_hand_ros2_sdk/src/
  $ pip install -r requirements.txt
```

## 使用 for Ubuntu
&ensp;&ensp; __使用前请先将 [setting.yaml](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml) 配置文件根据实际需求进行相应修改该.__
- 修改setting.yaml配置文件的密码，默认PASSWORD："12345678" 
默认密码为Ubuntu系统的密码，用户sdk自动开启CAN端口

&ensp;&ensp; __使用前请先将 [linker_hand.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py)文件按照实际灵巧手参数进行配置.__

- 启动SDK&ensp;&ensp;&ensp;&ensp;将linker_hand灵巧手的USB转CAN设备插入Ubuntu设备上  支持型号:L7/L10/L20/L21/L25
```bash
  # 开启CAN端口
  $ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB转CAN设备蓝色灯常亮状态
  $ cd linker_hand_ros2_sdk/
  $ colcon build --symlink-install
  $ source ./install/setup.bash
  $ sudo chmod a+x src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand.py
  $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  Current SDK version: 2.1.4
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set speed to [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set maximum torque to [200, 200, 200, 200, 200]
```

## 使用 for WIN+ROS2

&ensp;&ensp; __使用前请先将 [linker_hand.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py)文件按照实际灵巧手参数进行配置.__

- 启动SDK&ensp;&ensp;&ensp;&ensp;将linker_hand灵巧手的USB转CAN设备插入WIN系统设备上  支持型号:L7/L10/L20/L21/L25
- 注：安装好USB转CAN驱动后才可使用
```bash
  $ mkdir -p linker_hand_ros2_sdk/src
  $ cd linker_hand_ros2_sdk/src
  $ git clone https://github.com/linker-bot/linkerhand-ros2-sdk.git
  $ cd linker_hand_ros2_sdk/
  $ set PYTHONUTF8=1 # 设置环境变量为UTF-8编码
  $ colcon build --symlink-install
  $ call ./install/local_setup.bat
  $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py #先修改launch配置文件的CAN端口名称
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  Current SDK version: 2.1.4
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set speed to [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
  $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set maximum torque to [200, 200, 200, 200, 200]
```

- position与手指关节对照表
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
  O6:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲"]

  L6:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲"]

  L7:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]

  L10: ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]

  L20: ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]

  L21: ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

  L25: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

## 版本更新

- > ### release_2.1.9
 - 1、支持O6/L6版本灵巧手

- > ### release_2.1.8
 - 1、修复偶发撞帧问题

- > ### release_2.1.7
 - 1、修复已知问题
 - 2、将[Mujoco和PyBullet仿真](https://github.com/linker-bot/linkerhand-sim)移到仿真仓库中，减少SDK体量

- > ### release_2.1.6
  - 1、支持双CAN控制双灵巧手
  - 2、新增Mujoco仿真
  - 3、新增Pybullet仿真

- > ### release_1.0.3
  - 1、支持L20/L25版本灵巧手

- > ### release_1.0.2
  - 1、支持L10/O10版本灵巧手
  - 2、支持GUI控制L10/O10版本灵巧手
  - 3、增加支持压力传感器的LinkerHand波形图显示传感器状态
- > ### release_1.0.1
  - 1、支持L7/O7版本灵巧手
  - 2、支持GUI控制L7/O7版本灵巧手


## [示例](examples/)

&ensp;&ensp; __使用前请先将 [setting.yaml](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml) 配置文件根据实际需求进行相应修改该.__


## 通用
- [gui_control(图形界面控制)](图形界面控制)
图形界面控制可以通过滑动块控制LinkerHand灵巧手L10、L20各个关节独立运动。也可以通过添加按钮记录当前所有滑动块的数值，保存LinkerHand灵巧手当前各个关节运动状态。通过功能性按钮进行动作复现。    

使用gui_control控制LinkerHand灵巧手:
gui_control界面控制灵巧手需要启动linker_hand_sdk_ros，以topic的形式对LinkerHand灵巧手进行操作
开启ROS2 SDK后

&ensp;&ensp; __使用前请先将 [gui_control.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/gui_control/launch/gui_control.launch.py)文件按照实际灵巧手参数进行配置.__
```bash
# 新开终端
$ cd linker_hand_ros2_sdk/
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```
开启后会弹出UI界面。通过滑动条可控制相应LinkerHand灵巧手关节运动

## WIN+ROS2环境下使用GUI
&ensp;&ensp; __使用前请先将 [gui_control.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/gui_control/launch/gui_control.launch.py)文件按照实际灵巧手参数进行配置.__
```bash
# 新开终端
$ cd linker_hand_ros2_sdk/
$ call ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```

## L7
- [7001-action-group-show-ti(手指运动)](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/examples/L7/gesture/action-group-show-ti.py)

## L10
- [10001-action-group-show-normal(手指运动)](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/examples/L10/gesture/action-group-show-normal.py)


## Topic Document
[Linker Hand Topic Document](doc/Topic-Reference.md)

## Mujoco Pybullet仿真
 - [Mujoco和PyBullet仓库](https://github.com/linker-bot/linkerhand-sim)



