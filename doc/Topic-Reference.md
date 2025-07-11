# Linker Hand ROS SDK Topic Documentation

## Topic Overview

This document provides a detailed overview of the ROS Topic for the Linker Hand, including functions for controlling the hand's movements, retrieving sensor data, and setting operational parameters.

## Topic List
```bash
/cb_hand_setting_cmd # 设置linkerhand命令话题
/cb_left_hand_control_cmd # 控制左手运动话题 by range 0~255 (范围)
/cb_left_hand_control_cmd_arc # 控制左手运动话题 by arc -3.14~3.14 (弧度) 
/cb_left_hand_force # 左手压感数据显示话题
/cb_left_hand_matrix_touch # 左手矩阵压感数据显示话题 list(6x12)
/cb_left_hand_info  # 左手配置信息显示话题
/cb_left_hand_state # 左手状态显示话题 范围
/cb_left_hand_state_arc # 左手状态显示话题 弧度
/cb_right_hand_control_cmd # 控制右手运动话题 by range 0~255 (范围)
/cb_right_hand_control_cmd_arc # 控制右手运动话题 by arc -3.14~3.14 (弧度)
/cb_right_hand_force # 右手压感数据显示话题
/cb_right_hand_matrix_touch # 右手矩阵压感数据显示话题 list(6x12)
/cb_right_hand_info # 右手配置信息显示话题
/cb_right_hand_state # 右手状态显示话题 范围
/cb_right_hand_state_arc # 右手状态显示话题 弧度
```

### 获取手状态 Topic /cb_left_hand_state or /cb_right_hand_state
```bash

header: 
  seq: 211345
  stamp: 
    secs: 1744703535
    nsecs: 722361087
  frame_id: ''
name: 
  - joint71
  - joint72
  - joint73
  - joint77
  - joint75
  - joint76
  - joint77
  - joint78
  - joint79
  - joint80
  - joint81
  - joint82
  - joint83
  - joint84
  - joint88
  - joint86
  - joint87
  - joint88
  - joint89
  - joint90
position: [255.0, 132.0, 255.0, 255.0, 255.0, 255.0, 131.0, 127.0, 129.0, 127.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
**Description**: 
手指运动指定位置 数据格式 sensor_msgs/JointState 
**Parameters**:
- `position`: 手指joint当前状态 list(float) L7长度:7 L10长度:10 L20长度:20 L25长度:25 每个元素范围0~255 
---



### 获取矩阵式压感数据 Topic /cb_left_hand_matrix_touch or /cb_right_hand_matrix_touch 注：只第二代压力传感器
```bash
ros2 topic echo /cb_left_hand_matrix_touch
data: "{"thumb_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0]], "index_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,  0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "middle_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "ring_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "little_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]}"
```
**Description**: 
获取手指矩阵压感数据 数据格式 std_msgs/String Json
**Parameters**:
- `data`:
```bash
thumb_matrix：大拇指矩阵压力值 0~255
index_matrix：食指矩阵压力值 0~255
middle_matrix：中指矩阵压力值 0~255
ring_matrix：无名指矩阵压力值 0~255
little_matrix：小拇指矩阵压力值 0~255
```
---
### 获取LinkerHand配置信息 Topic /cb_left_hand_info or /cb_right_hand_info
```bash
ros2 topic echo /cb_right_hand_info
data: "{\"version\": [7, 0, 0, 0], \"hand_joint\": \"L21\", \"speed\": [1, 0, 0, 0, 0, 0,\
  \ 0, 0, 0, 0, 6, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0], \"current\"\
  : [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \"fault\": [[0,\
  \ 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,\
  \ 0, 0, 0, 0, 0]], \"motor_temperature\": [71, 52, 62, 46, 0, 65, 0, 50, 40, 0,\
  \ 0, 39, 0, 52, 41, 0, 0, 38, 0, 53, 41, 0, 0, 39, 0, 50, 40, 0, 0, 38], \"torque\"\
  : [16, 8, 3, 0, 0, 9, 0, 2, 0, 0, 0, 9, 0, 2, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 8,\
  \ 0, 0, 0, 8], \"is_touch\": true, \"touch_type\": 2, \"touch\": [0, 0, 0, 0, 0,\
  \ 0], \"finger_order\": [\"thumb_root\", \"index_finger_root\", \"middle_finger_root\"\
  , \"ring_finger_root\", \"little_finger_root\", \"thumb_abduction\", \"index_finger_abduction\"\
  , \"middle_finger_abduction\", \"ring_finger_abduction\", \"little_finger_abduction\"\
  , \"thumb_roll\", \"reserved\", \"reserved\", \"reserved\", \"reserved\", \"thumb_middle_joint\"\
  , \"reserved\", \"reserved\", \"reserved\", \"reserved\", \"thumb_tip\", \"index_finger_tip\"\
  , \"middle_finger_tip\", \"ring_finger_tip\", \"little_finger_tip\"]}"
```
**Description**: 
获取LinkerHand配置信息 数据格式 std_msgs/String for Json
**Parameters**:
- `version`: 手版本号 version[0]:表示L10 version[1]:表示版本 version[2]:表示批号 version[3]:76为左手82为右手 其他未内部编号
- `hand_joint`: L10 or L20 or L25等
- `speed`: 手指速度
- `current`: 手指当前电压 (若支持)
- `torque`: 手指扭矩 (若支持)
- `is_touch`: 是否有压力传感器
- `touch_type`: 传感器类型 (若支持)
- `touch`: 传感器数据 (若支持)
- `max_press_rco`: 最大电流
- `fault`: 电机故障 0 为正常 其他为故障
- `motor_temperature`: 当前电机温度
- `finger_order`: 当前灵巧手手指电机顺序

---



## range_to_arc 弧度角度对照表

获取和发送L10、L20的弧度值

topic:/cb_left_hand_state_arc and /cb_right_hand_state_arc 获取LinkerHand状态position为弧度值

topic:/cb_left_hand_control_cmd_arc 和 /cb_right_hand_control_cmd_arc 发布position弧度值控制LinkerHand手指运动

## 弧度与范围对照表

#---------------------------------------------------------------------------------------------------

L7灵巧手关节顺序 = ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]
### L7 L OK
l7_l_min = [0, 0, 0, 0, 0, 0, -0.52]
l7_l_max = [0.44, 1.43, 1.62, 1.62, 1.62, 1.62, 1.01]
l7_l_derict = [-1, -1, -1, -1, -1, -1, -1]
### L7 R OK (urdf后续会更改！！！)
l7_r_min = [0, -1.43, 0, 0, 0, 0, 0]
l7_r_max = [0.75, 0, 1.62, 1.62, 1.62, 1.62, 1.54]
l7_r_derict = [-1, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

L10灵巧手关节顺序 = ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]
### L10 L OK
l10_l_min = [0, 0, 0, 0, 0, 0, 0, -0.26, -0.26, -0.52]
l10_l_max = [1.45, 1.43, 1.62, 1.62, 1.62, 1.62, 0.26, 0, 0, 1.01]
l10_l_derict = [-1, -1, -1, -1, -1, -1, 0, -1, -1, -1]
### L10 R OK
l10_r_min = [0, 0, 0, 0, 0, 0, -0.26, 0, 0, -0.52]
l10_r_max = [0.75, 1.43, 1.62, 1.62, 1.62, 1.62, 0, 0.13, 0.26, 1.01]
l10_r_derict = [-1, -1, -1, -1, -1, -1, -1, 0, 0, -1]
#---------------------------------------------------------------------------------------------------

L20灵巧手关节顺序 = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]
### L20 L OK
l20_l_min = [0, 0, 0, 0, 0, -0.297, -0.26, -0.26, -0.26, -0.26, 0.122, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_l_max = [0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26, 1.78, 0, 0, 0, 0, 1.29, 1.08, 1.08, 1.08, 1.08]
l20_l_derict = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
### L20 R OK
l20_r_min = [0, 0, 0, 0, 0, -0.297, -0.26, -0.26, -0.26, -0.26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_r_max = [0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26, 1.78, 0, 0, 0, 0, 1.29, 1.08, 1.08, 1.08, 1.08]
l20_r_derict = [-1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

L21灵巧手关节顺序 = ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]
### L21 L OK
l21_l_min = [0, 0, 0, 0, 0, 0, 0, -0.18, -0.18, 0, -0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l21_l_max = [1, 1.57, 1.57, 1.57, 1.57, 1.6, 0.18, 0.18, 0.18, 0.18, 0.6, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l21_l_derict = [-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
### L21 R OK
l21_r_min = [0, 0, 0, 0, 0, 0, -0.18, -0.18, -0.18, -0.18, -0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l21_r_max = [1, 1.57, 1.57, 1.57, 1.57, 1.6, 0.18, 0.18, 0.18, 0.18, 0.6, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l21_r_derict = [-1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------



# Topic 示例
# L7
- L7右手握拳
```bash
ros2 topic pub /cb_right_hand_control_cmd sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: []
position: [75,115,0,0,0,0,42]
velocity: [255,255,255,255,255,255,255]
effort: []
"
```

- L7右手张开
```bash
ros2 topic pub /cb_right_hand_control_cmd sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: []
position: [255,255,255,255,255,255,34]
velocity: [255,255,255,255,255,255,255]
effort: [0.0, 0.0]
"
```

# L10
- L10右手握拳
```bash
ros2 topic pub /cb_right_hand_control_cmd sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: []
position: [73,75,0,0,0,0,110,110,120,78]
velocity: [255,255,255,255,255,255,255,255,255,255]
effort: []
"
```

- L10右手张开
```bash
ros2 topic pub /cb_right_hand_control_cmd sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: []
position: [255,210,255,255,255,255,110,110,120,33]
velocity: [255,255,255,255,255,255,255,255,255,255]
effort: [0.0, 0.0]
"
```
