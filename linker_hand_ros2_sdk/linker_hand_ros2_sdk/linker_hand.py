#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install
启动命令:ros2 run linker_hand_ros2_sdk linker_hand_sdk
'''
from re import A
import rclpy,sys                                     # ROS2 Python接口库
import time

from rclpy.node import Node                      # ROS2 节点类
from rclpy.clock import Clock
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState
import time, json, threading
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
        self.declare_parameter('modbus', "None")        

        # ros时间获取
        self.stamp_clock = Clock()
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.can = self.get_parameter('can').value
        self.modbus = self.get_parameter('modbus').value
        self.sdk_v = 2
        self.sleep_time = 0.005
        self.cmd_lock = False
        self.last_hand_post_cmd = None # 最新手指位置命令
        self.last_hand_vel_cmd = None # 最新手指速度命令
        self.last_hand_eff_cmd = None # 最新手指力矩命令

        self.last_hand_state = [-1] * 10
        self.last_hand_vel = [-1] * 10
        self.force = [[-1] * 5] * 4
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
        self.last_hand_info = {
            "version": [-1], # Dexterous hand version number
            "hand_joint": self.hand_joint, # Dexterous hand joint type
            "speed": [-1] * 10, # Current speed threshold of the dexterous hand
            "current": [-1] * 10, # Current of the dexterous hand
            "fault": [-1] * 10, # Current fault of the dexterous hand
            "motor_temperature": [-1] * 10, # Current motor temperature of the dexterous hand
            "torque": [-1] * 10, # Current torque of the dexterous hand
            "is_touch":self.is_touch,
            "touch_type": -1,
            "finger_order": None # Finger motor order
        }
        self.version = []
        self.touch_type = -1
        self.hz = 1.0/60.0

        self.hand_setting_sub = self.create_subscription(String,'/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        self._init_hand()
        time.sleep(1)
        self.run_count = 0 # 计数器，用于记录运行次数
        self.timer = self.create_timer(0.01, self.run)  # 100 Hz
        self.thread_pub_state = threading.Thread(target=self.pub_state)
        self.thread_pub_state.daemon = True
        self.thread_pub_state.start()

    def _init_hand(self):
        self.api = LinkerHandApi(hand_type=self.hand_type, hand_joint=self.hand_joint,modbus=self.modbus,can=self.can)
        time.sleep(0.1)
        self.touch_type = self.api.get_touch_type()
        self.hand_cmd_sub = self.create_subscription(JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb,10)
        self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state',10)
        self.hand_info_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        if self.is_touch == True:
            if self.touch_type == 2:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with matrix pressure sensing", color='green')
                self.matrix_touch_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
            elif self.touch_type != -1:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with pressure sensor", color="green")
                self.touch_pub = self.create_publisher(Float32MultiArray, f'/cb_{self.hand_type}_hand_force', 10)
            else:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Not equipped with any pressure sensors", color="red")
                self.is_touch = False
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
            torque = [255] * 10
            pose = [255, 200, 255, 255, 255, 255, 180, 180, 180, 41]
            speed = [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L20":
            pose = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
        elif self.hand_joint == "L21":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        elif self.hand_joint == "L25":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        if pose is not None:
            for i in range(1): 
                self.api.set_speed(speed=speed)
                time.sleep(0.1)
                self.api.set_torque(torque=torque)
                time.sleep(0.1)
                self.api.finger_move(pose=pose)
                time.sleep(0.1)

    def list_check(self,pose):
        if isinstance(pose, list) == False:
            return False
        if len(self.last_hand_post_cmd) != len(pose):
            return False
        return any(abs(self.last_hand_post_cmd - pose) >= 3 for self.last_hand_post_cmd, pose in zip(self.last_hand_post_cmd, pose))

    def hand_control_cb(self, msg):
        if self.last_hand_post_cmd == None or self.list_check(msg.position) == True:
            self.last_hand_post_cmd = msg.position
        if self.last_hand_vel_cmd == None or self.list_check(msg.velocity) == True:
            self.last_hand_vel_cmd = msg.velocity
        if self.last_hand_eff_cmd == None or self.list_check(msg.effort) == True:
            self.last_hand_eff_cmd = msg.effort

    def run(self):
        if self.sdk_v == 1:
            self.sleep_time = 0.009
        if self.hand_state_pub.get_subscription_count() > 0:
            # 优先获取手指状态并且发布
            self.last_hand_state = self.api.get_state()
            time.sleep(0.003)
            self.last_hand_vel = self.api.get_joint_speed()
            time.sleep(0.003)
        if self.cmd_lock == False:
            if self.last_hand_post_cmd != None:
                self.api.finger_move(pose=self.last_hand_post_cmd)
                self.last_hand_post_cmd = None
                time.sleep(0.003)
            if self.last_hand_vel_cmd != None:
                vel = list(self.last_hand_vel_cmd)
                if all(x == 0 for x in vel):
                    pass
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
                self.last_hand_vel_cmd = None
                time.sleep(0.003)
            if self.run_count == 3 and self.is_touch == True and self.touch_type == 1 and self.touch_pub.get_subscription_count() > 0:
                """单点式压力传感器"""
                self.force = self.api.get_force()
            if self.is_touch == True and self.touch_type == 2 and self.matrix_touch_pub.get_subscription_count() > 0:
                """矩阵式压力传感器"""
                if self.run_count == 3:
                    self.matrix_dic["thumb_matrix"] = self.api.get_thumb_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 4:
                    self.matrix_dic["index_matrix"] = self.api.get_index_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 5:
                    self.matrix_dic["middle_matrix"] = self.api.get_middle_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 6:
                    self.matrix_dic["ring_matrix"] = self.api.get_ring_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 7:
                    self.matrix_dic["little_matrix"] = self.api.get_little_matrix_touch(sleep_time=self.sleep_time).tolist()
            if self.run_count == 8 and self.hand_info_pub.get_subscription_count() > 0:
                """手部信息"""
                self.last_hand_info = {
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
            if self.run_count == 9:
                self.run_count = 0
            self.run_count += 1
            time.sleep(0.005)


    def pub_state(self):
        while True:
            if self.hand_state_pub.get_subscription_count() > 0:
                msg = self.joint_state_msg(self.last_hand_state, self.last_hand_vel)
                self.hand_state_pub.publish(msg)
            if self.is_touch == True and self.touch_type == 1 and self.touch_pub.get_subscription_count() > 0:
                msg = Float32MultiArray()
                msg.data = [float(val) for sublist in self.force for val in sublist]
                self.touch_pub.publish(msg)
            if self.is_touch == True and self.touch_type == 2 and self.matrix_touch_pub.get_subscription_count() > 0:
                msg = String()
                current_time = self.stamp_clock.now()
                # 提取 secs 和 nsecs
                t_sec = current_time.to_msg().sec
                t_nanosec = current_time.to_msg().nanosec
                self.matrix_dic["stamp"]["sec"] = t_sec
                self.matrix_dic["stamp"]["nanosec"] = t_nanosec
                msg.data = json.dumps(self.matrix_dic)
                self.matrix_touch_pub.publish(msg)
            if self.hand_info_pub.get_subscription_count() > 0:
                msg = String()
                msg.data = json.dumps(self.last_hand_info)
                self.hand_info_pub.publish(msg)
            time.sleep(self.hz)

    

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
            self.cmd_lock = True
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
            self.cmd_lock = False
        finally:
            self.cmd_lock = False


    def close_can(self):
        self.api.open_can.close_can(can=self.can)
        sys.exit(0)

        
def main(args=None):
    try:
        rclpy.init(args=args)
        node = LinkerHand("linker_hand_sdk")
        embedded_version = node.embedded_version
        if len(embedded_version) == 3 or node.hand_joint.upper() == "O6" or node.hand_joint.upper() == "L6" or node.hand_joint.upper() == "G20":
            ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
            node.sdk_v = 2
        elif len(embedded_version) == 6 and node.hand_joint == "L10":
            ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
            node.sdk_v = 2
        elif len(embedded_version) > 4 and ((embedded_version[0]==10 and embedded_version[4]>35) or (embedded_version[0]==7 and embedded_version[4]>50) or (embedded_version[0] == 6)):
            ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
            node.sdk_v = 2
        else:
            ColorMsg(msg=f"SDK V1", color="green")
            node.sdk_v = 1
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        # node.close_can()         # 关闭 CAN 或其他硬件资源
        # node.destroy_node()      # 销毁 ROS 节点
        # rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")
