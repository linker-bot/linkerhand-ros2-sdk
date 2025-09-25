#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install
启动命令:ros2 run linker_hand_ros2_sdk linker_hand_sdk
'''
import rclpy,math,sys                                     # ROS2 Python接口库
import subprocess
import time

from rclpy.node import Node                      # ROS2 节点类
import numpy as np
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState
import time,threading, json
from datetime import datetime
from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi
from linker_hand_ros2_sdk.LinkerHand.utils.color_msg import ColorMsg
from linker_hand_ros2_sdk.LinkerHand.utils.open_can import OpenCan


class LinkerHand(Node):
    def __init__(self, name):
        super().__init__(name)
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L6')
        self.declare_parameter('is_touch', False)
        self.declare_parameter('can', 'can0')
        
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.can = self.get_parameter('can').value
        self.last_left_hand_move_pose = []
        self.last_right_hand_move_pose = []
        self.vel = []
        self.version = []
        self.touch_type = -1
        self.t_force = [-1] * 5
        self.last_hand_info = {
            "version": [], # Dexterous hand version number
            "hand_joint": self.hand_joint, # Dexterous hand joint type
            "speed": [], # Current speed threshold of the dexterous hand
            "current": [], # Current of the dexterous hand
            "fault": [], # Current fault of the dexterous hand
            "motor_temperature": [], # Current motor temperature of the dexterous hand
            "torque": [], # Current torque of the dexterous hand
            "is_touch":self.is_touch,
            "touch_type": self.touch_type,
            "finger_order": [] # Finger motor order
        }
        self.last_hand_state = {
            "state": [-1] * 5,
            "vel": [-1] * 5
        }
        self.matrix_dic = {
            "thumb_matrix":[[-1] * 12 for _ in range(6)],
            "index_matrix":[[-1] * 12 for _ in range(6)],
            "middle_matrix":[[-1] * 12 for _ in range(6)],
            "ring_matrix":[[-1] * 12 for _ in range(6)],
            "little_matrix":[[-1] * 12 for _ in range(6)]
        }
        self.last_hand_matrix_touch = String()
        self.last_hand_touch = String()
        self.open_can = OpenCan()
        self.hand_setting_sub = self.create_subscription(String,'/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        self.last_process_time = 0
        self.max_hz = 15
        self.min_interval = 1.0 / self.max_hz
        self.lock = threading.Lock()
        self.init_hand(hand_type=self.hand_type)

        
        

    def init_hand(self,hand_type):
        if hand_type == "left":
            self.api = LinkerHandApi(hand_type=hand_type, hand_joint=self.hand_joint,can=self.can)
            self.open_can.open_can(self.can)
            time.sleep(0.1)
            self.touch_type = self.api.get_touch_type()
            self.hand_cmd_sub = self.create_subscription(JointState, '/cb_left_hand_control_cmd', self.left_hand_control_cb,10)
            self.hand_cmd_arc_sub = self.create_subscription(JointState, '/cb_left_hand_control_cmd_arc', self.left_hand_control_arc_cb,10)
            self.hand_state_pub = self.create_publisher(JointState, '/cb_left_hand_state',10)
            self.hand_state_arc_pub = self.create_publisher(JointState, '/cb_left_hand_state_arc',10)
            self.hand_info_pub = self.create_publisher(String, '/cb_left_hand_info', 10)
            if self.is_touch == True:
                if self.touch_type == 2:
                    ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with matrix pressure sensing", color='green')
                    self.matrix_touch_pub = self.create_publisher(String, '/cb_left_hand_matrix_touch', 10)
                elif self.touch_type != -1:
                    ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with pressure sensor", color="green")
                    self.touch_pub = self.create_publisher(Float32MultiArray, '/cb_left_hand_force', 10)
                else:
                    ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Not equipped with any pressure sensors", color="red")
        elif hand_type == "right":
            self.api = LinkerHandApi(hand_type=hand_type, hand_joint=self.hand_joint,can=self.can)
            self.open_can.open_can(self.can)
            time.sleep(0.1)
            self.touch_type = self.api.get_touch_type()
            self.hand_cmd_sub = self.create_subscription(JointState, '/cb_right_hand_control_cmd', self.right_hand_control_cb,10)
            self.hand_cmd_arc_sub = self.create_subscription(JointState, '/cb_right_hand_control_cmd_arc', self.right_hand_control_arc_cb,10)
            self.hand_state_pub = self.create_publisher(JointState, '/cb_right_hand_state',10)
            self.hand_state_arc_pub = self.create_publisher(JointState, '/cb_right_hand_state_arc',10)
            self.hand_info_pub = self.create_publisher(String, '/cb_right_hand_info', 10)
            if self.is_touch == True:
                if self.touch_type == 2:
                    ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with matrix pressure sensing", color='green')
                    self.matrix_touch_pub = self.create_publisher(String, '/cb_right_hand_matrix_touch', 10)
                elif self.touch_type != -1:
                    ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with pressure sensor", color="green")
                    self.touch_pub = self.create_publisher(Float32MultiArray, '/cb_right_hand_force', 10)
                else:
                    ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Not equipped with any pressure sensors", color="red")
        self.embedded_version = self.api.get_embedded_version()
        pose = None
        torque = [200, 200, 200, 200, 200]
        speed = [200, 250, 250, 250, 250]
        if self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6" or self.hand_joint.upper() == "L6P":
            pose = [200, 255, 255, 255, 255, 180]
            torque = [250, 250, 250, 250, 250, 250]
            # O6 最大速度阈值
            speed = [200, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L7":
            # The data length of L7 is 7, reinitialize here
            pose = [255, 200, 255, 255, 255, 255, 180]
            torque = [250, 250, 250, 250, 250, 250, 250]
            speed = [120, 250, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L10":
            pose = [255, 200, 255, 255, 255, 255, 180, 180, 180, 41]
            speed = [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L20":
            pose = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
        elif self.hand_joint == "L21":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        elif self.hand_joint == "L25":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        if pose is not None:
            for i in range(1): # 循环设置3次，防止数据丢失
                self.api.set_speed(speed=speed)
                time.sleep(0.1)
                self.api.set_torque(torque=torque)
                time.sleep(0.1)
                self.api.finger_move(pose=pose)
                time.sleep(0.1)


    def run(self):
        self.thread_get_state = threading.Thread(target=self._get_hand_state)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()
        # self.thread_get_state.join()
        
        self.thread_get_info = threading.Thread(target=self.get_hand_info)
        self.thread_get_info.daemon = True
        self.thread_get_info.start()
        # self.thread_get_info.join()
        if self.is_touch == True:
            if self.touch_type == 2:
                self.thread_get_matrix_touch = threading.Thread(target=self.get_matrix_touch)
                self.thread_get_matrix_touch.daemon = True
                self.thread_get_matrix_touch.start()
                # self.thread_get_matrix_touch.join()
            elif self.touch_type != -1 and self.touch_type != 2:
                self.thread_get_touch = threading.Thread(target=self.get_hand_touch)
                self.thread_get_touch.daemon = True
                self.thread_get_touch.start()
                # self.thread_get_touch.join()


    def run_v2(self):
        self.thread_get_all_state = threading.Thread(target=self.get_all_state_v2)
        self.thread_get_all_state.daemon = True
        self.thread_get_all_state.start()
        self.thread_pub_all_state = threading.Thread(target=self.get_pub_state_v2)
        self.thread_pub_all_state.daemon = True
        self.thread_pub_all_state.start()

    def get_all_state_v2(self):
        count = 0
        while True:
            self._get_hand_state_v2()
            if count % 3 == 0 and self.is_touch == True and self.touch_type == 2: # 如果配置了压感并且压感验证了类型为矩阵式压感
                self.get_matrix_touch_v2()
            if count % 5 == 0:
                self.get_hand_info_v2()
            if count == 10:
                count = 0
            count += 1
            time.sleep(0.01)

    def get_pub_state_v2(self):
        m_t = String()
        while True:
            self.pub_hand_state(hand_state=self.last_hand_state)
            self.pub_hand_info(dic=self.last_hand_info)
            m_t.data = json.dumps(self.matrix_dic)
            if self.is_touch == True and self.touch_type == 2: # 如果配置了压感并且压感验证了类型为矩阵式压感
                self.matrix_touch_pub.publish(m_t)
            time.sleep(0.033)


    def _get_hand_state(self):
        hand_state = {
            'state':[],
            'vel':[]
        }
        while True:
            if self.hand_state_pub.get_subscription_count() > 0 or self.hand_state_arc_pub.get_subscription_count() > 0:
                state = self.api.get_state()
                vel = self.api.get_joint_speed()
                # if self.embedded_version[0] == 6 and self.embedded_version[4] == 16:
                #     hand_state['state'] = [state[0], state[5], state[1], state[2], state[3], state[4]]
                #     hand_state['vel'] = [vel[0], vel[5], vel[1], vel[2], vel[3], vel[4]]
                # else:
                hand_state['state'] = state
                hand_state['vel'] = vel
                self.pub_hand_state(hand_state=hand_state)
                time.sleep(0.02)

    def _get_hand_state_v2(self):
        if self.hand_state_pub.get_subscription_count() > 0 or self.hand_state_arc_pub.get_subscription_count() > 0:
            state = self.api.get_state()
            vel = self.api.get_joint_speed()
            self.last_hand_state['state'] = state
            self.last_hand_state['vel'] = vel



    def pub_hand_state(self,hand_state):
        state = hand_state['state']
        vel = hand_state['vel']
        if state[0] == -1:
            return
        if self.hand_type == "left":
            s_a = self.api.range_to_arc_left(state,self.hand_joint)
        if self.hand_type == "right":
            s_a = self.api.range_to_arc_right(state,self.hand_joint)
        
        state_arc = [round(x, 2) for x in s_a]
        
        if state == None:
            return
        if all(x == 0 for x in  vel):
            self.hand_state_pub.publish(self.joint_state_msg(state,[0]*len(state)))
            self.hand_state_arc_pub.publish(self.joint_state_msg(state_arc,[0]*len(state)))
        else:
            self.hand_state_pub.publish(self.joint_state_msg(state,vel))
            self.hand_state_arc_pub.publish(self.joint_state_msg(state_arc,vel))

    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state
    
    def get_hand_info(self):
        while True:
            if self.hand_info_pub.get_subscription_count() > 0:
                data = {
                    "version": self.embedded_version, # Dexterous hand version number
                    "hand_joint": self.hand_joint, # Dexterous hand joint type
                    "speed": self.api.get_speed(), # Current speed threshold of the dexterous hand
                    "current": self.api.get_current(), # Current of the dexterous hand
                    "fault": self.api.get_fault(), # Current fault of the dexterous hand
                    "motor_temperature": self.api.get_temperature(), # Current motor temperature of the dexterous hand
                    "torque": self.api.get_torque(), # Current torque of the dexterous hand
                    "is_touch":self.is_touch,
                    "touch_type": self.touch_type,
                    "finger_order": self.api.get_finger_order() # Finger motor order
                }
                #self.last_hand_info = data
                self.pub_hand_info(dic=data)
            time.sleep(0.3)

    def get_hand_info_v2(self):
        if self.hand_info_pub.get_subscription_count() > 0:
            data = {
                "version": self.embedded_version, # Dexterous hand version number
                "hand_joint": self.hand_joint, # Dexterous hand joint type
                "speed": self.api.get_speed(), # Current speed threshold of the dexterous hand
                "current": self.api.get_current(), # Current of the dexterous hand
                "fault": self.api.get_fault(), # Current fault of the dexterous hand
                "motor_temperature": self.api.get_temperature(), # Current motor temperature of the dexterous hand
                "torque": self.api.get_torque(), # Current torque of the dexterous hand
                "is_touch":self.is_touch,
                "touch_type": self.touch_type,
                "finger_order": self.api.get_finger_order() # Finger motor order
            }
            self.last_hand_info = data
            # self.pub_hand_info(dic=data)

    def pub_hand_info(self,dic):
        msg = String()
        msg.data = json.dumps(dic)
        self.hand_info_pub.publish(msg)

    def get_hand_touch(self):
        while True:
            if self.touch_pub.get_subscription_count() > 0:
                if self.is_touch == True:
                    #self.touch_type = self.api.get_touch_type()
                    if self.touch_type == 2:
                        break
                        self.t_force = self.api.get_touch()
                    elif self.touch_type != -1:
                        force = self.api.get_force()
                        self.t_force = [item for sublist in force for item in sublist]
                else:
                    self.touch_type = -1
                if self.is_touch == True:
                    if self.touch_type != 2 and self.touch_type !=-1:
                        t_force = self.t_force
                        force_msg = Float32MultiArray()
                        force_msg.data = t_force
                        self.last_hand_touch = force_msg
                    self.touch_pub.publish(force_msg)
            time.sleep(0.04)

    def get_matrix_touch(self):
        while True:
            if self.matrix_touch_pub.get_subscription_count() > 0:
                if self.touch_type == 2:
                    thumb_matrix, index_matrix , middle_matrix , ring_matrix , little_matrix = self.api.get_matrix_touch()
                    matrix_dic = {
                        "thumb_matrix":thumb_matrix.tolist(),
                        "index_matrix":index_matrix.tolist(),
                        "middle_matrix":middle_matrix.tolist(),
                        "ring_matrix":ring_matrix.tolist(),
                        "little_matrix":little_matrix.tolist()
                    }
                    # print(matrix_dic,flush=True)
                    m_t = String()
                    m_t.data = json.dumps(matrix_dic)
                    self.matrix_touch_pub.publish(m_t)
            time.sleep(0.15)

    def get_matrix_touch_v2(self):
        if self.matrix_touch_pub.get_subscription_count() > 0:
            if self.touch_type == 2:
                thumb_matrix, index_matrix , middle_matrix , ring_matrix , little_matrix = self.api.get_matrix_touch_v2()
                matrix_dic = {
                    "thumb_matrix":thumb_matrix.tolist(),
                    "index_matrix":index_matrix.tolist(),
                    "middle_matrix":middle_matrix.tolist(),
                    "ring_matrix":ring_matrix.tolist(),
                    "little_matrix":little_matrix.tolist()
                }
                self.matrix_dic = matrix_dic
                

    def left_hand_control_cb(self,msg):
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        tmp_pose = [0] * 6
        pose = list(msg.position)
        if len(pose) == 0:
            return
        else:
            '''左手接收控制topic回调 for range'''
            self.api.finger_move(pose=pose)
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if (str(self.hand_joint).upper() == "O6" or str(self.hand_joint).upper() == "L6" or str(self.hand_joint).upper() == "L6P") and len(vel) == 6:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def left_hand_control_arc_cb(self,msg):
        if self.hand_cmd_sub.get_publisher_count() > 0:
            return
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''左手接收控制topic回调 for arc'''
        pose_range = self.api.arc_to_range_left(msg.position,self.hand_joint)
        self.api.finger_move(pose=list(pose_range))
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if (str(self.hand_joint).upper() == "O6" or str(self.hand_joint).upper() == "L6" or str(self.hand_joint).upper() == "L6P") and len(vel) == 6:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def right_hand_control_cb(self,msg):
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''右手接收控制topic回调 for range'''
        pose = list(msg.position)
        tmp_pose = [0] * 6
        if len(pose) == 0:
            return
        else:
            '''右手接收控制topic回调 for range'''
            self.api.finger_move(pose=pose)
        self.api.finger_move(pose=list(msg.position))
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if (str(self.hand_joint).upper() == "O6" or str(self.hand_joint).upper() == "L6" or str(self.hand_joint).upper() == "L6P") and len(vel) == 6:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def right_hand_control_arc_cb(self,msg):
        if self.hand_cmd_sub.get_publisher_count() > 0:
            return
        now = time.time()
        if now - self.last_process_time < self.min_interval:
            return  # 丢弃当前帧，限频处理
        self.last_process_time = now
        '''右手接收控制topic回调 for arc'''
        pose_range = self.api.arc_to_range_right(msg.position,self.hand_joint)
        print(pose_range, flush=True)
        self.api.finger_move(pose=list(pose_range))
        vel = list(msg.velocity)
        self.vel = vel
        if all(x == 0 for x in vel):
            return
        else:
            if (str(self.hand_joint).upper() == "O6" or str(self.hand_joint).upper() == "L6" or str(self.hand_joint).upper() == "L6P") and len(vel) == 6:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L7" and len(vel) == 7:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L10" and len(vel) == 10:
                speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L20" and len(vel) == 20:
                speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L21" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)
            elif self.hand_joint == "L25" and len(vel) == 25:
                speed = vel
                self.api.set_joint_speed(speed=speed)

    def hand_setting_cb(self,msg):
        '''控制命令回调'''
        data = json.loads(msg.data)
        print(f"Received setting command: {data['setting_cmd']}",flush=True)
        try:
            if data["params"]["hand_type"] == "left":
                hand = self.api
                hand_left = True
            elif data["params"]["hand_type"] == "right":
                hand = self.api
                hand_right = True
            else:
                print("Please specify the hand part to be set",flush=True)
                return
            # Set maximum torque
            if data["setting_cmd"] == "set_max_torque_limits": # Set maximum torque
                torque = list(data["params"]["torque"])
                hand.set_torque(torque=torque)
                
            if data["setting_cmd"] == "set_speed": # Set speed
                if isinstance(data["params"]["speed"], list) == True:
                    speed = data["params"]["speed"]
                    hand.set_speed(speed=speed)
                else:
                    ColorMsg(msg=f"Speed parameter error, speed must be a list", color="red")
            if data["setting_cmd"] == "clear_faults": # Clear faults
                if hand_left == True and self.hand_joint == "L10" :
                    ColorMsg(msg=f"L10 left hand cannot clear faults")
                elif hand_right == True and self.hand_joint == "L10" :
                    ColorMsg(msg=f"L10 right hand cannot clear faults")
                else:
                    hand.clear_faults()
            if data["setting_cmd"] == "get_faults": # Get faults
                f = hand.get_fault()
                ColorMsg(msg=f"Get faults: {f}")
            if data["setting_cmd"] == "electric_current": # Get current
                ColorMsg(msg=f"Get current: {hand.get_current()}")
            if data["setting_cmd"] == "set_electric_current": # Set current
                if isinstance(data["params"]["current"], list) == True:
                    hand.set_current(data["params"]["current"])
            if data["setting_cmd"] == "show_fun_table": # Get faults
                f = hand.show_fun_table()
        except:
            print("命令参数错误")


    def close_can(self):
        self.open_can.close_can(can=self.can)
        sys.exit(0)

        
def main(args=None):
    rclpy.init(args=args)
    node = LinkerHand("linker_hand_sdk")
    embedded_version = node.embedded_version
    try:
        if len(embedded_version) == 3:
            ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
            node.run_v2()
        elif len(embedded_version) > 4 and ((embedded_version[0]==10 and embedded_version[4]>35) or (embedded_version[0]==7 and embedded_version[4]>50) or (embedded_version[0] == 6)):
            ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
            node.run_v2()
        else:
            ColorMsg(msg=f"SDK V1", color="green")
            node.run()
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        node.close_can()         # 关闭 CAN 或其他硬件资源
        node.destroy_node()      # 销毁 ROS 节点
        rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")
