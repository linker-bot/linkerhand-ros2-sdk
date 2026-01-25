#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

from re import A
import rclpy,sys                                     # ROS2 Python接口库
import time
import argparse
import numpy as np
from rclpy.node import Node                      # ROS2 节点类
from rclpy.clock import Clock
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState, PointCloud2, PointField
import time, json, threading
from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi
from linker_hand_ros2_sdk.LinkerHand.utils.color_msg import ColorMsg
from linker_hand_ros2_sdk.LinkerHand.utils.open_can import OpenCan

class LinkerHandAdvancedL10(Node):
    def __init__(self, name, hand_type, can, is_touch):
        super().__init__(name)       
        self.hand_type = hand_type
        self.hand_joint = "L10"
        if is_touch == "true":
            self.is_touch = True
        else:
            self.is_touch = False
        self.can = can
        self.modbus = "None"
        time.sleep(0.1)
        self._check_linker_hand_type()
        self.last_hand_post_cmd = None # 最新手指位置命令
        self.last_hand_vel_cmd = None # 最新手指速度命令
        self.last_hand_eff_cmd = None # 最新手指力矩命令
        self.count = 0 # 循环计数器
        self.matrix_dic = {
            "stamp":{
                "sec": 0,
                "nanosec": 0,
            },
            "thumb_matrix":[[-1] * 6 for _ in range(12)],
            "index_matrix":[[-1] * 6 for _ in range(12)],
            "middle_matrix":[[-1] * 6 for _ in range(12)],
            "ring_matrix":[[-1] * 6 for _ in range(12)],
            "little_matrix":[[-1] * 6 for _ in range(12)]
        }
        # 压感矩阵合值，单位g 克
        self.matrix_mass_dic = {
            "stamp":{
                "secs": 0,
                "nsecs": 0,
            },
            "thumb_mass":[-1],
            "index_mass":[-1],
            "middle_mass":[-1],
            "ring_mass":[-1],
            "little_mass":[-1]
        }
        self.hz = 1.0/60.0
        # ros时间获取
        self.stamp_clock = Clock()
        self._init_hand()
        time.sleep(2)
        self.timer = self.create_timer(self.hz, self.run)  # 100 Hz


    def _check_linker_hand_type(self):
        if self.modbus != "None":
            ColorMsg(msg=f"Modbus暂不支持", color="red")
            sys.exit(0)
        if self.hand_joint.upper() != "L10":
            ColorMsg(msg=f"L10以外其他Linker Hand暂不支持", color="red")
            sys.exit(0)


    def _init_hand(self):
        self.api = LinkerHandApi(hand_type=self.hand_type, hand_joint=self.hand_joint,modbus=self.modbus,can=self.can)
        time.sleep(0.1)
        self.touch_type = self.api.get_touch_type()
        self.hand_cmd_sub = self.create_subscription(JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb,10)
        self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state',10)
        if self.is_touch == True:
            if self.touch_type > 1:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with matrix pressure sensing", color='green')
                self.matrix_touch_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
                self.matrix_touch_pub_pc = self.create_publisher(PointCloud2, f'/cb_{self.hand_type}_hand_matrix_touch_pc', 10)
                self.matrix_touch_mass_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
            elif self.touch_type != -1:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with pressure sensor", color="green")
                self.touch_pub = self.create_publisher(Float32MultiArray, f'/cb_{self.hand_type}_hand_force', 10)
            else:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Not equipped with any pressure sensors", color="red")
                self.is_touch = False
        self.embedded_version = self.api.get_embedded_version()
        if self.hand_joint.upper() == "L10":
            pose = [255, 200, 255, 255, 255, 255, 180, 180, 180, 41]
            torque = [255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
            speed = [255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
            self.api.set_speed(speed=speed)
            time.sleep(0.1)
            self.api.set_torque(torque=torque)
            time.sleep(0.1)
            self.api.finger_move(pose=pose)
            time.sleep(0.1)
        self.serial_number = self.api.get_serial_number()

    def hand_control_cb(self, msg):
        if self.last_hand_post_cmd == None or self.list_check(msg.position) == True:
            self.last_hand_post_cmd = msg.position
        if self.last_hand_vel_cmd == None or self.list_check(msg.velocity) == True:
            self.last_hand_vel_cmd = msg.velocity
        if self.last_hand_eff_cmd == None or self.list_check(msg.effort) == True:
            self.last_hand_eff_cmd = msg.effort
    
    def list_check(self,pose):
        if isinstance(pose, list) == False:
            return False
        if len(self.last_hand_post_cmd) != len(pose):
            return False
        return any(abs(self.last_hand_post_cmd - pose) >= 3 for self.last_hand_post_cmd, pose in zip(self.last_hand_post_cmd, pose))
    
    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.api.get_finger_order()
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state

    def run(self):
        # 执行手控制指令
        if self.last_hand_post_cmd != None:
            self.api.finger_move(pose=self.last_hand_post_cmd)
            self.last_hand_post_cmd = None
        # 优先获取手指状态并且发布
        self.last_hand_state = self.api.get_state()
        self.last_hand_vel = [0.0] * len(self.last_hand_state)
        # 发布手状态
        msg_state = self.joint_state_msg(self.last_hand_state)
        self.hand_state_pub.publish(msg_state)
        # 获取压感数据
        if self.is_touch == True:
            self.matrix_dic["thumb_matrix"] = self.api.get_thumb_matrix_touch(sleep_time=0.003).tolist()
            self.matrix_dic["index_matrix"] = self.api.get_index_matrix_touch(sleep_time=0.004).tolist()
            self.matrix_dic["middle_matrix"] = self.api.get_middle_matrix_touch(sleep_time=0.003).tolist()
            self.matrix_dic["ring_matrix"] = self.api.get_ring_matrix_touch(sleep_time=0.004).tolist()
            self.matrix_dic["little_matrix"] = self.api.get_little_matrix_touch(sleep_time=0.004).tolist()
            # 发布矩阵压感数据JSON格式
            self.pub_matrix_dic()
            # 发布矩阵压感和值JSON格式
            self.pub_matrix_mass(dic=self.matrix_dic)
            # 发布矩阵压感点云格式
            self.pub_matrix_point_cloud()
        

    def pub_matrix_dic(self):
        """发布矩阵数据JSON格式"""
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        self.matrix_dic["stamp"]["secs"] = t_secs
        self.matrix_dic["stamp"]["nsecs"] = t_nsecs
        msg.data = json.dumps(self.matrix_dic)
        self.matrix_touch_pub.publish(msg)

    def pub_matrix_mass(self, dic):
        """发布矩阵数据合值 单位g 克 JSON格式"""
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        self.matrix_mass_dic["stamp"]["secs"] = t_secs
        self.matrix_mass_dic["stamp"]["nsecs"] = t_nsecs
        self.matrix_mass_dic["unit"] = "g"
        self.matrix_mass_dic["thumb_mass"] = sum(sum(row) for row in dic["thumb_matrix"])
        self.matrix_mass_dic["index_mass"] = sum(sum(row) for row in dic["index_matrix"])
        self.matrix_mass_dic["middle_mass"] = sum(sum(row) for row in dic["middle_matrix"])
        self.matrix_mass_dic["ring_mass"] = sum(sum(row) for row in dic["ring_matrix"])
        self.matrix_mass_dic["little_mass"] = sum(sum(row) for row in dic["little_matrix"])
        msg.data = json.dumps(self.matrix_mass_dic)
        self.matrix_touch_mass_pub.publish(msg)

    def pub_matrix_point_cloud(self):
        tmp_dic = self.matrix_dic.copy()
        del tmp_dic['stamp']               # 去掉时间戳字段
        all_matrices = list(tmp_dic.values())  # 5 帧，每帧 6×12=72 个数 or 5 帧，每帧 4×10=40 个数 列x行
        # 摊平到一维
        flat_list = [v for frame in all_matrices for v in frame]  
        flat = np.concatenate([np.asarray(np.clip(c, 0, 255), dtype=np.uint8) for c in flat_list])
        fields = [PointField(name='val', offset=0, datatype=PointField.UINT8, count=1)]
        pc = PointCloud2()
        pc.header.stamp =  self.get_clock().now().to_msg()
        pc.header.frame_id = ''   # 可改成你需要的坐标系
        pc.height = 1
        pc.width = flat.size         # 360
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = 1            # 1 个 float32
        pc.row_step = pc.point_step * pc.width
        pc.data = flat.tobytes()     # 1440 字节
        self.matrix_touch_pub_pc.publish(pc)


    def close_can(self):
        self.api.open_can.close_can(can=self.can)
        sys.exit(0)


def main(args=None):
    '''
    本节点用于收集手指状态和压感数据。
    '/cb_{self.hand_type}_hand_control_cmd' 话题类型为 sensor_msgs/msg/JointState 控制话题，限制 30Hz
    /cb_{self.hand_type}_hand_state 话题类型为 sensor_msgs/msg/JointState 30Hz
    '/cb_{self.hand_type}_hand_matrix_touch' 话题类型为 std_msgs/msg/String 30Hz
    '/cb_{self.hand_type}_hand_matrix_touch_pc', 10)
                self.matrix_touch_mass_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
    启动命令:
    ros2 run linker_hand_ros2_sdk linker_hand_advanced_l10 --hand_type left --can can0 --is_touch true
    '''
    try:
        rclpy.init(args=args)
        parser = argparse.ArgumentParser()
        parser.add_argument('--hand_type', required=True)
        parser.add_argument('--can',        required=True)
        parser.add_argument('--is_touch',   choices=['true','false'], required=True)

        args = parser.parse_args()
        node = LinkerHandAdvancedL10(name="linker_hand_advanced_l10",hand_type=args.hand_type,can=args.can,is_touch=args.is_touch)
        embedded_version = node.embedded_version
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        # node.close_can()         # 关闭 CAN 或其他硬件资源
        # node.destroy_node()      # 销毁 ROS 节点
        # rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")