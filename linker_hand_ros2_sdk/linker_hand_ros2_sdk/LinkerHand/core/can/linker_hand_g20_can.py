#!/usr/bin/env python3
import can
import time, sys, os
import threading
import numpy as np
from enum import Enum
from utils.open_can import OpenCan
from can.exceptions import CanError
from utils.color_msg import ColorMsg
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)
"""
拇指41: [拇指侧摆, 拇指横摆, 拇指根部, 预留, 预留, 拇指尖部]
食指42: [食指侧摆, 预留, 食指根部, 预留, 预留, 食指末端]
中指43: [中指侧摆, 预留, 中指根部, 预留, 预留, 中指末端]
无名指44: [无名指侧摆, 预留, 无名指根部, 预留, 预留, 无名指末端]
小指45: [小指侧摆, 预留, 小指根部, 预留, 预留, 小指末端]
"""
CMD_MAP = [
    "拇指根部", 
    "食指根部", 
    "中指根部", 
    "无名指根部",
    "小指根部",
    "拇指侧摆",
    "食指侧摆",
    "中指侧摆",
    "无名指侧摆",
    "小指侧摆",
    "拇指横摆",
    "预留",
    "预留",
    "预留",
    "预留",
    "拇指尖部",
    "食指末端",
    "中指末端",
    "无名指末端",
    "小指末端"
]

class FrameProperty(Enum):
    # 手指运动控制 - 并联型控制指令（控制所有手指同一关节）
    ROLL_POS = 0x01  # 横滚关节位置
    YAW_POS = 0x02  # 航向关节位置
    ROOT1_POS = 0x03  # 指根1关节位置
    ROOT2_POS = 0x04  # 指根2关节位置
    ROOT3_POS = 0x05  # 指根3关节位置
    TIP_POS = 0x06  # 指尖关节位置
    
    # 关节速度指令
    ROLL_SPEED = 0x09  # 横滚关节速度
    YAW_SPEED = 0x0A  # 航向关节速度
    ROOT1_SPEED = 0x0B  # 指根1关节速度
    ROOT2_SPEED = 0x0C  # 指根2关节速度
    ROOT3_SPEED = 0x0D  # 指根3关节速度
    TIP_SPEED = 0x0E  # 指尖关节速度
    
    # 关节扭矩指令
    ROLL_TORQUE = 0x11  # 横滚关节扭矩
    YAW_TORQUE = 0x12  # 航向关节扭矩
    ROOT1_TORQUE = 0x13  # 指根1关节扭矩
    ROOT2_TORQUE = 0x14  # 指根2关节扭矩
    ROOT3_TORQUE = 0x15  # 指根3关节扭矩
    TIP_TORQUE = 0x16  # 指尖关节扭矩
    
    # 关节故障码
    ROLL_FAULT = 0x19  # 横滚关节故障码
    YAW_FAULT = 0x1A  # 航向关节故障码
    ROOT1_FAULT = 0x1B  # 指根1关节故障码
    ROOT2_FAULT = 0x1C  # 指根2关节故障码
    ROOT3_FAULT = 0x1D  # 指根3关节故障码
    TIP_FAULT = 0x1E  # 指尖关节故障码
    
    # 关节温度
    ROLL_TEMPERATURE = 0x21  # 横滚关节过温保护阈值
    YAW_TEMPERATURE = 0x22  # 航向关节过温保护阈值
    ROOT1_TEMPERATURE = 0x23  # 指根1关节过温保护阈值
    ROOT2_TEMPERATURE = 0x24  # 指根2关节过温保护阈值
    ROOT3_TEMPERATURE = 0x25  # 指根3关节过温保护阈值
    TIP_TEMPERATURE = 0x26  # 指尖关节过温保护阈值
    
    # 手指运动控制 - 串联型控制指令（控制同一手指所有关节）
    THUMB_POS = 0x41  # 大拇指指关节位置
    INDEX_POS = 0x42  # 食指关节位置
    MIDDLE_POS = 0x43  # 中指关节位置
    RING_POS = 0x44  # 无名指关节位置
    LITTLE_POS = 0x45  # 小拇指关节位置
    
    # 手指速度
    THUMB_SPEED = 0x49  # 大拇指速度
    INDEX_SPEED = 0x4A  # 食指速度
    MIDDLE_SPEED = 0x4B  # 中指速度
    RING_SPEED = 0x4C  # 无名指速度
    LITTLE_SPEED = 0x4D  # 小拇指速度
    
    # 手指扭矩
    THUMB_TORQUE = 0x51  # 大拇指扭矩
    INDEX_TORQUE = 0x52  # 食指扭矩
    MIDDLE_TORQUE = 0x53  # 中指扭矩
    RING_TORQUE = 0x54  # 无名指扭矩
    LITTLE_TORQUE = 0x55  # 小拇指扭矩
    
    # 手指故障码
    THUMB_FAULT = 0x59  # 大拇指故障码
    INDEX_FAULT = 0x5A  # 食指故障码
    MIDDLE_FAULT = 0x5B  # 中指故障码
    RING_FAULT = 0x5C  # 无名指故障码
    LITTLE_FAULT = 0x5D  # 小拇指故障码
    
    # 手指温度
    THUMB_TEMPERATURE = 0x61  # 大拇指过温保护阈值
    INDEX_TEMPERATURE = 0x62  # 食指过温保护阈值
    MIDDLE_TEMPERATURE = 0x63  # 中指过温保护阈值
    RING_TEMPERATURE = 0x64  # 无名指过温保护阈值
    LITTLE_TEMPERATURE = 0x65  # 小拇指过温保护阈值
    
    # 手指运动控制 - 合并指令区域
    FINGER_SPEED = 0x81  # 设置手指速度
    FINGER_TORQUE = 0x82  # 设置手指输出扭矩
    FINGER_FAULT = 0x83  # 清除手指故障及故障码
    FINGER_TEMPERATURE = 0x84  # 手指各关节温度
    
    # 指尖传感器数据
    HAND_NORMAL_FORCE = 0x90  # 五指法向压力
    HAND_TANGENTIAL_FORCE = 0x91  # 五指切向压力
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # 五指切向方向
    HAND_APPROACH_INC = 0x93  # 五指接近感应
    
    # 手指所有数据
    THUMB_ALL_DATA = 0x98  # 大拇指所有数据
    INDEX_ALL_DATA = 0x99  # 食指所有数据
    MIDDLE_ALL_DATA = 0x9A  # 中指所有数据
    RING_ALL_DATA = 0x9B  # 无名指所有数据
    LITTLE_ALL_DATA = 0x9C  # 小拇指所有数据
    
    # 触觉传感器
    TOUCH_SENSOR_TYPE = 0xB0  # 触觉传感器类型
    THUMB_TOUCH = 0xB1  # 大拇指触觉传感
    INDEX_TOUCH = 0xB2  # 食指触觉传感
    MIDDLE_TOUCH = 0xB3  # 中指触觉传感
    RING_TOUCH = 0xB4  # 无名指触觉传感
    LITTLE_TOUCH = 0xB5  # 小拇指触觉传感
    PALM_TOUCH = 0xB6  # 手掌指触觉传感
    
    # 查询指令
    HAND_UID_GET = 0xC0  # 唯一标识码查询
    HAND_HARDWARE_VERSION_GET = 0xC1  # 硬件版本查询
    HAND_SOFTWARE_VERSION_GET = 0xC2  # 软件版本查询
    HAND_COMM_ID_GET = 0xC3  # 设备id查询
    HAND_STRUCT_VERSION_GET = 0xC4  # 结构版本号查询
    
    # 出厂指令
    HOST_CMD_HAND_ERASE_POS_CALI = 0xCD  # 擦除位置校准值
    HAND_COMM_ID_SET = 0xD1  # 通信ID设置
    HAND_UID_SET = 0xF0  # 唯一标识码设置

class LinkerHandG20Can:
    def __init__(self, can_channel='can0', baudrate=1000000, can_id=0x28, yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)

        self.running = True
        
        # 初始化数据存储变量
        self.last_thumb_pos, self.last_index_pos, self.last_ring_pos, self.last_middle_pos, self.last_little_pos = None, None, None, None, None
        self.last_root1, self.last_yaw, self.last_roll, self.last_root2, self.last_tip = None, None, None, None, None
        
        # 并联控制数据存储
        self.x01, self.x02, self.x03, self.x04, self.x05, self.x06 = [], [], [], [], [], []
        self.x09, self.x0A, self.x0B, self.x0C, self.x0D, self.x0E = [], [], [], [], [], []
        self.x11, self.x12, self.x13, self.x14, self.x15, self.x16 = [], [], [], [], [], []
        self.x19, self.x1A, self.x1B, self.x1C, self.x1D, self.x1E = [], [], [], [], [], []
        self.x21, self.x22, self.x23, self.x24, self.x25, self.x26 = [], [], [], [], [], []
        
        # 串联控制数据存储
        self.x41, self.x42, self.x43, self.x44, self.x45 = [], [], [], [], []
        self.x49, self.x4A, self.x4B, self.x4C, self.x4D = [0] * 6, [0] * 6, [0] * 6, [0] * 6, [0] * 6
        self.x51, self.x52, self.x53, self.x54, self.x55 = [], [], [], [], []
        self.x59, self.x5A, self.x5B, self.x5C, self.x5D = [], [], [], [], []
        self.x61, self.x62, self.x63, self.x64, self.x65 = [], [], [], [], []
        
        # 合并指令区域数据存储
        self.x81, self.x82, self.x83, self.x84 = [], [], [], []
        
        # 传感器数据存储
        self.x90, self.x91, self.x92, self.x93 = [], [], [], []
        self.x98, self.x99, self.x9A, self.x9B, self.x9C = [], [], [], [], []
        self.xB0, self.xB1, self.xB2, self.xB3, self.xB4, self.xB5, self.xB6 = [], [], [], [], [], [], []
        self.normal_force, self.tangential_force, self.tangential_force_dir, self.approach_inc = [[-1] * 5 for _ in range(4)]
        
        # 查询指令数据存储
        self.xC0, self.xC1, self.xC2, self.xC3, self.xC4 = [], [], [], [], []
        self.serial_number = []
        self.serial_number_map = {
            0: 0,
            1: 1,
            2: 2,
            3: 3,
        }
        # 触觉传感器矩阵数据
        self.thumb_matrix = np.full((12, 6), -1)
        self.index_matrix = np.full((12, 6), -1)
        self.middle_matrix = np.full((12, 6), -1)
        self.ring_matrix = np.full((12, 6), -1)
        self.little_matrix = np.full((12, 6), -1)
        self.matrix_map = {
            0: 0, 16: 1, 32: 2, 48: 3, 64: 4, 80: 5,
            96: 6, 112: 7, 128: 8, 144: 9, 160: 10, 176: 11,
        }
        # 全掌触觉数据缓存
        self.thumb_matrix_palm = np.full((23, 9), -1)
        self.thumb_matrix_palm_tmp = []
        self.thumb_matrix_palm_mass = [-1, -1, -1]

        self.index_matrix_palm = np.full((23, 9), -1)
        self.index_matrix_palm_tmp = []
        self.index_matrix_palm_mass = [-1, -1, -1]

        self.middle_matrix_palm = np.full((23, 9), -1)
        self.middle_matrix_palm_tmp = []
        self.middle_matrix_palm_mass = [-1, -1, -1]

        self.ring_matrix_palm = np.full((23, 9), -1)
        self.ring_matrix_palm_tmp = []
        self.ring_matrix_palm_mass = [-1, -1, -1]

        self.little_matrix_palm = np.full((23, 9), -1)
        self.little_matrix_palm_tmp = []
        self.little_matrix_palm_mass = [-1, -1, -1]

        self.palm_matrix_palm = np.full((28, 20), -1)
        self.palm_matrix_palm_tmp = []
        self.palm_matrix_palm_mass = [-1, -1]

        self.bus = self.init_can_bus(channel=self.can_channel, baudrate=baudrate)
        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self._check_touch_type()

        self.xB0 = self.get_touch_sensor_type() # 获取触觉传感器类型,如果返回值为5：TSSP_JZG(全手掌，指尖11x9,指中6x9,指根6x9,数据以23行9列形式返回，五指各有3个合力值。手掌20x28，手掌有两个合力值，分为上掌上半部分和下半部分。)

    def _check_touch_type(self):
        '''根据SN编码判断压感类型'''
        self.sn = self.get_serial_number()
        time.sleep(0.1)
        if self.sn != "-1":
            parts = self.sn.split("-")
            if parts[4] == "A":
                self.touch_type = 1
            elif parts[4] == "B":
                self.touch_type = 2
                self.touch_code = 0xC6  # 6*12
            elif parts[4] == "J":
                self.touch_type = 3
            elif parts[4] == "F":
                self.touch_type = 4
                self.touch_code = 0xA4 # 4*10
            elif parts[4] == "Z":
                self.touch_type = -1
        else:
            # 如果没有SN编码则根据返回数据进行判断
            self.touch_type = self.get_touch_type()

    def init_can_bus(self, channel, baudrate):
        """
        尝试按优先级连接 CAN 总线，并实现回退机制。
        """
        # --- 统一异常处理块开始 ---
        try:
            if sys.platform == "linux":
                # Linux 优先级：1. socketcan
                try:
                    self.open_can.open_can(self.can_channel)
                    # 尝试 socketcan
                    bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=baudrate)
                    ColorMsg(msg=f"成功连接: interface='socketcan', channel='{channel}'", color="green")
                    return bus
                except CanError as e:
                    # 如果 socketcan 失败，可以考虑在这里尝试其他 Linux 接口 (如 'pcan')
                    ColorMsg(msg=f"socketcan 接口连接失败: {e}", color="yellow")
                    raise # 重新抛出异常，让外层 try 捕获
            elif sys.platform == "win32":
                # Windows 优先级：1. pcan
                try:
                    bus = can.interface.Bus(channel=channel, interface='pcan', bitrate=baudrate)
                    ColorMsg(msg=f"成功连接: interface='pcan', channel='{channel}'", color="green")
                    return bus
                except CanError as e:
                    ColorMsg(msg=f"pcan 接口连接失败，尝试回退到 'candle': {e}", color="yellow")
                # Windows 优先级：2. candle (回退方法)
                try:
                    bus = can.Bus(interface="candle", channel=channel, bitrate=baudrate)
                    ColorMsg(msg=f"成功连接: interface='candle', channel='{channel}'", color="green")
                    return bus
                except CanError as e:
                    ColorMsg(msg=f"candle 接口连接失败: {e}", color="yellow")
                    raise # 两个接口都失败，抛出异常
            else:
                raise EnvironmentError("Unsupported platform for CAN interface")
        # --- 统一异常处理块结束 ---
        except Exception as e:
            # 如果任何一个接口尝试失败并抛出异常（包括 EnvironmentError）
            ColorMsg(msg=f"致命错误：所有 CAN 接口连接尝试均失败或平台不受支持。请检查设备连接或驱动安装和配置文件中CAN参数的配置。\n错误详情: {e}", color="red")
            # 保持 raise 动作，将错误信息传递给调用者，避免程序继续运行
            raise

    def send_command(self, frame_property, data_list, sleep_time=0.003):
        """
        发送指令到CAN总线
        :param frame_property: 数据帧属性
        :param data_list: 数据载荷
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Failed to send message: {e}")
            self.open_can.open_can(self.can_channel)
            time.sleep(1)
            self.is_can = self.open_can.is_can_up_sysfs(interface=self.can_channel)
            time.sleep(1)
            if self.is_can:
                self.bus = can.interface.Bus(channel=self.can_channel, interface="socketcan", bitrate=self.baudrate)
            else:
                print("Reconnecting CAN devices ....")
        time.sleep(sleep_time)

    def receive_response(self):
        """
        接收并处理CAN总线响应消息
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")

    def process_response(self, msg):
        """
        处理CAN响应消息
        """
        if msg.arbitration_id == self.can_id:
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if len(list(response_data)) == 0:
                    return
            # 并联控制指令响应
            if frame_type == 0x01: self.x01 = list(response_data)
            elif frame_type == 0x02: self.x02 = list(response_data)
            elif frame_type == 0x03: self.x03 = list(response_data)
            elif frame_type == 0x04: self.x04 = list(response_data)
            elif frame_type == 0x05: self.x05 = list(response_data)
            elif frame_type == 0x06: self.x06 = list(response_data)
            elif frame_type == 0x09: self.x09 = list(response_data)
            elif frame_type == 0x0A: self.x0A = list(response_data)
            elif frame_type == 0x0B: self.x0B = list(response_data)
            elif frame_type == 0x0C: self.x0C = list(response_data)
            elif frame_type == 0x0D: self.x0D = list(response_data)
            elif frame_type == 0x0E: self.x0E = list(response_data)
            elif frame_type == 0x11: self.x11 = list(response_data)
            elif frame_type == 0x12: self.x12 = list(response_data)
            elif frame_type == 0x13: self.x13 = list(response_data)
            elif frame_type == 0x14: self.x14 = list(response_data)
            elif frame_type == 0x15: self.x15 = list(response_data)
            elif frame_type == 0x16: self.x16 = list(response_data)
            elif frame_type == 0x19: self.x19 = list(response_data)
            elif frame_type == 0x1A: self.x1A = list(response_data)
            elif frame_type == 0x1B: self.x1B = list(response_data)
            elif frame_type == 0x1C: self.x1C = list(response_data)
            elif frame_type == 0x1D: self.x1D = list(response_data)
            elif frame_type == 0x1E: self.x1E = list(response_data)
            elif frame_type == 0x21: self.x21 = list(response_data)
            elif frame_type == 0x22: self.x22 = list(response_data)
            elif frame_type == 0x23: self.x23 = list(response_data)
            elif frame_type == 0x24: self.x24 = list(response_data)
            elif frame_type == 0x25: self.x25 = list(response_data)
            elif frame_type == 0x26: self.x26 = list(response_data)
            
            # 串联控制指令响应
            elif frame_type == 0x41: self.x41 = list(response_data)
            elif frame_type == 0x42: self.x42 = list(response_data)
            elif frame_type == 0x43: self.x43 = list(response_data)
            elif frame_type == 0x44: self.x44 = list(response_data)
            elif frame_type == 0x45: self.x45 = list(response_data)
            elif frame_type == 0x49: self.x49 = list(response_data)
            elif frame_type == 0x4A: self.x4A = list(response_data)
            elif frame_type == 0x4B: self.x4B = list(response_data)
            elif frame_type == 0x4C: self.x4C = list(response_data)
            elif frame_type == 0x4D: self.x4D = list(response_data)
            elif frame_type == 0x51: self.x51 = list(response_data)
            elif frame_type == 0x52: self.x52 = list(response_data)
            elif frame_type == 0x53: self.x53 = list(response_data)
            elif frame_type == 0x54: self.x54 = list(response_data)
            elif frame_type == 0x55: self.x55 = list(response_data)
            elif frame_type == 0x59: self.x59 = list(response_data)
            elif frame_type == 0x5A: self.x5A = list(response_data)
            elif frame_type == 0x5B: self.x5B = list(response_data)
            elif frame_type == 0x5C: self.x5C = list(response_data)
            elif frame_type == 0x5D: self.x5D = list(response_data)
            elif frame_type == 0x61: self.x61 = list(response_data)
            elif frame_type == 0x62: self.x62 = list(response_data)
            elif frame_type == 0x63: self.x63 = list(response_data)
            elif frame_type == 0x64: self.x64 = list(response_data)
            elif frame_type == 0x65: self.x65 = list(response_data)
            
            # 合并指令区域响应
            elif frame_type == 0x81: self.x81 = list(response_data)
            elif frame_type == 0x82: self.x82 = list(response_data)
            elif frame_type == 0x83: self.x83 = list(response_data)
            elif frame_type == 0x84: self.x84 = list(response_data)
            
            # 传感器数据响应
            elif frame_type == 0x90: self.x90 = list(response_data)
            elif frame_type == 0x91: self.x91 = list(response_data)
            elif frame_type == 0x92: self.x92 = list(response_data)
            elif frame_type == 0x93: self.x93 = list(response_data)
            elif frame_type == 0x98: self.x98 = list(response_data)
            elif frame_type == 0x99: self.x99 = list(response_data)
            elif frame_type == 0x9A: self.x9A = list(response_data)
            elif frame_type == 0x9B: self.x9B = list(response_data)
            elif frame_type == 0x9C: self.x9C = list(response_data)
            
            # 触觉传感器响应
            elif frame_type == 0xB0: self.xB0 = list(response_data)
            elif frame_type == 0xB1:
                d = list(response_data)
                if self.xB0[0] == 5:
                    """全掌矩阵"""
                    self.thumb_matrix_palm_tmp.append(d) # 将返回帧存储到缓存
                    if len(d) == 4 and d[0] == 22 and d[1] == 7: # 如果是最后一帧
                        self.thumb_matrix_palm = self.build_matrix(self.thumb_matrix_palm_tmp) # 将帧数据排列成23行9列矩阵
                        self.thumb_matrix_palm_tmp=[] # 重置缓存数据
                    if len(d) == 7 and d[0] == 255:
                        self.thumb_matrix_palm_mass = self.build_matrix_mass(d) # [指尖合力值, 指中合力值, 指根合力值]
                else:
                    if len(d) == 2:
                        self.xB1 = d
                    elif len(d) == 7:
                        index = self.matrix_map.get(d[0])
                        if index is not None:
                            self.thumb_matrix[index] = d[1:]
                        
            elif frame_type == 0xB2: 
                d = list(response_data)
                if self.xB0[0] == 5:
                    """全掌矩阵"""
                    self.index_matrix_palm_tmp.append(d) # 将返回帧存储到缓存
                    if len(d) == 4 and d[0] == 22 and d[1] == 7: # 如果是最后一帧
                        self.index_matrix_palm = self.build_matrix(self.index_matrix_palm_tmp) # 将帧数据排列成23行9列矩阵
                        self.index_matrix_palm_tmp=[] # 重置缓存数据
                    if len(d) == 7 and d[0] == 255:
                        self.index_matrix_palm_mass = self.build_matrix_mass(d) # [指尖合力值, 指中合力值, 指根合力值]
                else:
                    if len(d) == 2:
                        self.xB2 = d
                    elif len(d) == 7:
                        index = self.matrix_map.get(d[0])
                        if index is not None:
                            self.index_matrix[index] = d[1:]
            elif frame_type == 0xB3: 
                d = list(response_data)
                if self.xB0[0] == 5:
                    """全掌矩阵"""
                    self.middle_matrix_palm_tmp.append(d) # 将返回帧存储到缓存
                    if len(d) == 4 and d[0] == 22 and d[1] == 7: # 如果是最后一帧
                        self.middle_matrix_palm = self.build_matrix(self.middle_matrix_palm_tmp) # 将帧数据排列成23行9列矩阵
                        self.middle_matrix_palm_tmp=[] # 重置缓存数据
                    if len(d) == 7 and d[0] == 255:
                        self.middle_matrix_palm_mass = self.build_matrix_mass(d) # [指尖合力值, 指中合力值, 指根合力值]
                else:
                    if len(d) == 2:
                        self.xB3 = d
                    elif len(d) == 7:
                        index = self.matrix_map.get(d[0])
                        if index is not None:
                            self.middle_matrix[index] = d[1:]
            elif frame_type == 0xB4: 
                d = list(response_data)
                if self.xB0[0] == 5:
                    """全掌矩阵"""
                    self.ring_matrix_palm_tmp.append(d) # 将返回帧存储到缓存
                    if len(d) == 4 and d[0] == 22 and d[1] == 7: # 如果是最后一帧
                        self.ring_matrix_palm = self.build_matrix(self.ring_matrix_palm_tmp) # 将帧数据排列成23行9列矩阵
                        self.ring_matrix_palm_tmp=[] # 重置缓存数据
                    if len(d) == 7 and d[0] == 255:
                        self.ring_matrix_palm_mass = self.build_matrix_mass(d) # [指尖合力值, 指中合力值, 指根合力值]
                else:
                    if len(d) == 2:
                        self.xB4 = d
                    elif len(d) == 7:
                        index = self.matrix_map.get(d[0])
                        if index is not None:
                            self.ring_matrix[index] = d[1:]
            elif frame_type == 0xB5: 
                d = list(response_data)
                if self.xB0[0] == 5:
                    """全掌矩阵"""
                    self.little_matrix_palm_tmp.append(d) # 将返回帧存储到缓存
                    if len(d) == 4 and d[0] == 22 and d[1] == 7: # 如果是最后一帧
                        self.little_matrix_palm = self.build_matrix(self.little_matrix_palm_tmp) # 将帧数据排列成23行9列矩阵
                        self.little_matrix_palm_tmp=[] # 重置缓存数据
                    if len(d) == 7 and d[0] == 255:
                        self.little_matrix_palm_mass = self.build_matrix_mass(d) # [指尖合力值, 指中合力值, 指根合力值]
                else:
                    if len(d) == 2:
                        self.xB5 = d
                    elif len(d) == 7:
                        index = self.matrix_map.get(d[0])
                        if index is not None:
                            self.little_matrix[index] = d[1:]
            elif frame_type == 0xB6:
                d = list(response_data)
                if self.xB0[0] == 5:
                    """全掌矩阵"""
                    self.palm_matrix_palm_tmp.append(d) # 将返回帧存储到缓存
                    if len(d) == 7 and d[0] == 27 and d[1] == 5: # 如果是最后一帧
                        self.palm_matrix_palm = self.build_matrix(self.palm_matrix_palm_tmp) # 将帧数据排列成23行9列矩阵
                        self.palm_matrix_palm_tmp=[] # 重置缓存数据
                    if len(d) == 5 and d[0] == 255:
                        self.palm_matrix_palm_mass = self.build_matrix_mass(d)
                else:
                    self.xB6 = d

            
            # 查询指令响应
            # elif frame_type == 0xC0: self.xC0 = list(response_data)
            elif frame_type == 0xC0:
                d = list(response_data)
                index = self.serial_number_map.get(d[0])
                if index is not None:
                    self.serial_number=self.serial_number + d[1:]
                else:
                    self.serial_number=self.serial_number + [-1] * 6
            elif frame_type == 0xC1: self.xC1 = list(response_data)
            elif frame_type == 0xC2: self.xC2 = list(response_data)
            elif frame_type == 0xC3: self.xC3 = list(response_data)
            elif frame_type == 0xC4: self.xC4 = list(response_data)


    
    def build_matrix(self, data, r=23, c=9):
        rows, cols = r, c
        matrix = np.full((rows, cols), -1)
        
        for item in data:
            if len(item) == 4:
                # 最后一行：从列坐标开始放
                row, col, v1, v2 = item
                start_col = col  # 改为 col，而不是 col+1
                values = [v1, v2]
                
                cur_row, cur_col = row, start_col
                for val in values:
                    if cur_col >= cols:
                        cur_row += 1
                        cur_col = 0
                    if cur_row < rows:
                        matrix[cur_row][cur_col] = val
                        cur_col += 1
                break
            
            elif len(item) == 7:
                # 普通行：从列坐标开始放
                row, col, v1, v2, v3, v4, v5 = item
                start_col = col  # 改为 col，而不是 col+1
                values = [v1, v2, v3, v4, v5]
                
                cur_row, cur_col = row, start_col
                for val in values:
                    if cur_col >= cols:
                        cur_row += 1
                        cur_col = 0
                    if cur_row < rows:
                        matrix[cur_row][cur_col] = val
                        cur_col += 1
        
        return matrix
    
    def build_matrix_mass(self, hex_data):
        """
        处理返回的和力值的帧数据,手指返回合力值长度为3，[指尖，指中，指根]
        手掌返回为长度为2。[上半部,下半部]
        解析 CAN 数据（支持 5 字节或 7 字节）
        
        参数:
            hex_data: 十六进制列表，如 [0xFF, 0x27, 0x02, 0x51, 0x05, 0x20, 0x04] (7字节)
                    或 [0xFF, 0x92, 0x09, 0xF8, 0x00] (5字节)
        
        返回:
            (id, values) 其中 id 是 int，values 是包含 int 的列表（2个或3个）
        """
        if len(hex_data) not in [5, 7]:
            raise ValueError(f"数据长度不支持，需要 5 或 7 字节，实际: {len(hex_data)}")
        
        # 获取 ID（第一个字节）
        can_id = hex_data[0]
        
        # 计算有多少组数据（每组2字节）
        data_bytes = hex_data[1:]  # 去掉 ID
        num_values = len(data_bytes) // 2
        
        # 解析数据组（小端序）
        values = []
        for i in range(num_values):
            low_byte = data_bytes[i*2]      # 低位字节
            high_byte = data_bytes[i*2 + 1]  # 高位字节
            # 小端拼接：低位 + 高位<<8
            value = low_byte | (high_byte << 8)
            values.append(value)
        
        return values


    # 并联控制指令方法
    def set_roll_positions(self, joint_ranges):
        """设置所有手指横滚关节位置"""
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    
    def set_yaw_positions(self, joint_ranges):
        """设置所有手指航向关节位置"""
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    
    def set_root1_positions(self, joint_ranges):
        """设置所有手指指根1关节位置"""
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    
    def set_root2_positions(self, joint_ranges):
        """设置所有手指指根2关节位置"""
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    
    def set_root3_positions(self, joint_ranges):
        """设置所有手指指根3关节位置"""
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    
    def set_tip_positions(self, joint_ranges=[80]*5):
        """设置所有手指指尖关节位置"""
        self.send_command(FrameProperty.TIP_POS, joint_ranges)

    # 串联控制指令方法
    def set_thumb_positions(self, joint_ranges):
        """设置大拇指所有关节位置"""
        self.send_command(FrameProperty.THUMB_POS, joint_ranges)
    
    def set_index_positions(self, joint_ranges):
        """设置食指所有关节位置"""
        self.send_command(FrameProperty.INDEX_POS, joint_ranges)
    
    def set_middle_positions(self, joint_ranges):
        """设置中指所有关节位置"""
        self.send_command(FrameProperty.MIDDLE_POS, joint_ranges)
    
    def set_ring_positions(self, joint_ranges):
        """设置无名指所有关节位置"""
        self.send_command(FrameProperty.RING_POS, joint_ranges)
    
    def set_little_positions(self, joint_ranges):
        """设置小拇指所有关节位置"""
        self.send_command(FrameProperty.LITTLE_POS, joint_ranges)

    # 扭矩设置方法
    def set_thumb_torque(self, torque_values):
        """设置大拇指扭矩"""
        self.send_command(FrameProperty.THUMB_TORQUE, torque_values)
    
    def set_index_torque(self, torque_values):
        """设置食指扭矩"""
        self.send_command(FrameProperty.INDEX_TORQUE, torque_values)
    
    def set_middle_torque(self, torque_values):
        """设置中指扭矩"""
        self.send_command(FrameProperty.MIDDLE_TORQUE, torque_values)
    
    def set_ring_torque(self, torque_values):
        """设置无名指扭矩"""
        self.send_command(FrameProperty.RING_TORQUE, torque_values)
    
    def set_little_torque(self, torque_values):
        """设置小拇指扭矩"""
        self.send_command(FrameProperty.LITTLE_TORQUE, torque_values)

    # 速度设置方法
    def set_thumb_speed(self, speed_values):
        """设置大拇指速度"""
        self.send_command(FrameProperty.THUMB_SPEED, speed_values)
    
    def set_index_speed(self, speed_values):
        """设置食指速度"""
        self.send_command(FrameProperty.INDEX_SPEED, speed_values)
    
    def set_middle_speed(self, speed_values):
        """设置中指速度"""
        self.send_command(FrameProperty.MIDDLE_SPEED, speed_values)
    
    def set_ring_speed(self, speed_values):
        """设置无名指速度"""
        self.send_command(FrameProperty.RING_SPEED, speed_values)
    
    def set_little_speed(self, speed_values):
        """设置小拇指速度"""
        self.send_command(FrameProperty.LITTLE_SPEED, speed_values)

    # 查询方法
    def get_thumb_positions(self):
        """获取大拇指所有关节当前位置"""
        self.send_command(FrameProperty.THUMB_POS, [])
        return self.x41
    
    def get_index_positions(self):
        """获取食指所有关节当前位置"""
        self.send_command(FrameProperty.INDEX_POS, [])
        return self.x42
    
    def get_middle_positions(self):
        """获取中指所有关节当前位置"""
        self.send_command(FrameProperty.MIDDLE_POS, [])
        return self.x43
    
    def get_ring_positions(self):
        """获取无名指所有关节当前位置"""
        self.send_command(FrameProperty.RING_POS, [])
        return self.x44
    
    def get_little_positions(self):
        """获取小拇指所有关节当前位置"""
        self.send_command(FrameProperty.LITTLE_POS, [])
        return self.x45


    def get_thumb_speed(self):
        """获取大拇指速度"""
        self.send_command(FrameProperty.THUMB_SPEED, [])
    
    def get_index_speed(self):
        """获取食指速度"""
        self.send_command(FrameProperty.INDEX_SPEED, [])
    
    def get_middle_speed(self):
        """获取中指速度"""
        self.send_command(FrameProperty.MIDDLE_SPEED, [])
    
    def get_ring_speed(self):
        """获取无名指速度"""
        self.send_command(FrameProperty.RING_SPEED, [])
    
    def get_little_speed(self):
        """获取小拇指速度"""
        self.send_command(FrameProperty.LITTLE_SPEED, [])

    def get_thumb_torque(self):
        """获取大拇指扭矩"""
        self.send_command(FrameProperty.THUMB_TORQUE, [])
    
    def get_index_torque(self):
        """获取食指扭矩"""
        self.send_command(FrameProperty.INDEX_TORQUE, [])
    
    def get_middle_torque(self):
        """获取中指扭矩"""
        self.send_command(FrameProperty.MIDDLE_TORQUE, [])
    
    def get_ring_torque(self):
        """获取无名指扭矩"""
        self.send_command(FrameProperty.RING_TORQUE, [])
    
    def get_little_torque(self):
        """获取小拇指扭矩"""
        self.send_command(FrameProperty.LITTLE_TORQUE, [])

    def get_thumb_fault(self):
        """获取大拇指所有关节故障码"""
        self.send_command(FrameProperty.THUMB_FAULT, [])
        return self.x59
    
    def get_index_fault(self):
        """获取食指所有关节故障码"""
        self.send_command(FrameProperty.INDEX_FAULT, [])
        return self.x5A
    
    def get_middle_fault(self):
        """获取中指所有关节故障码"""
        self.send_command(FrameProperty.MIDDLE_FAULT, [])
        return self.x5B
    
    def get_ring_fault(self):
        """获取无名指所有关节故障码"""
        self.send_command(FrameProperty.RING_FAULT, [])
        return self.x5C
    
    def get_little_fault(self):
        """获取小拇指所有关节故障码"""
        self.send_command(FrameProperty.LITTLE_FAULT, [])
        return self.x5D

    def get_thumb_temperature(self):
        """获取大拇指所有关节当前温度"""
        self.send_command(FrameProperty.THUMB_TEMPERATURE, [])
        return self.x61
    
    def get_index_temperature(self):
        """获取食指所有关节当前温度"""
        self.send_command(FrameProperty.INDEX_TEMPERATURE, [])
        return self.x62
    
    def get_middle_temperature(self):
        """获取中指所有关节当前温度"""
        self.send_command(FrameProperty.MIDDLE_TEMPERATURE, [])
        return self.x63
    
    def get_ring_temperature(self):
        """获取无名指所有关节当前温度"""
        self.send_command(FrameProperty.RING_TEMPERATURE, [])
        return self.x64
    
    def get_little_temperature(self):
        """获取小拇指所有关节当前温度"""
        self.send_command(FrameProperty.LITTLE_TEMPERATURE, [])
        return self.x65

    # 合并指令区域方法    
    def set_finger_speed(self, speed_values):
        """设置手指速度"""
        self.send_command(FrameProperty.FINGER_SPEED, speed_values)
    
    def set_finger_torque(self, torque_values):
        """设置手指输出扭矩"""
        self.send_command(FrameProperty.FINGER_TORQUE, torque_values)
    
    def clear_finger_faults(self, finger_mask=[1, 1, 1, 1, 1]):
        """清除手指故障及故障码"""
        self.send_command(FrameProperty.FINGER_FAULT, finger_mask)
        return self.x83
    
    def get_finger_temperature(self):
        """获取手指各关节温度"""
        self.send_command(FrameProperty.FINGER_TEMPERATURE, [])
        return self.x84

    # 传感器数据获取方法
    def get_normal_force(self):
        """获取五指法向力"""
        self.send_command(FrameProperty.HAND_NORMAL_FORCE, [])
        return self.x90
    
    def get_tangential_force(self):
        """获取五指切向力"""
        self.send_command(FrameProperty.HAND_TANGENTIAL_FORCE, [])
        return self.x91
    
    def get_tangential_force_dir(self):
        """获取五指切向力方向"""
        self.send_command(FrameProperty.HAND_TANGENTIAL_FORCE_DIR, [])
        return self.x92
    
    def get_approach_inc(self):
        """获取五指接近感应"""
        self.send_command(FrameProperty.HAND_APPROACH_INC, [])
        return self.x93
    
    def get_force(self):
        '''Get pressure sensor data'''
        return [self.x90,self.x91,self.x92,self.x93]

    # 触觉传感器方法
    def get_touch_sensor_type(self):
        """获取触觉传感器类型 暂仅支持G20"""
        self.send_command(FrameProperty.TOUCH_SENSOR_TYPE, [])
        return self.xB0[0]
    
    def get_thumb_touch(self):
        """获取大拇指触觉传感数据"""
        if self.xB0[0] == 5:
            d = [23, 9, 1]
            sleep_time = 0.015
        else:
            d = [0xC6]
            sleep_time = 0.007
        self.send_command(FrameProperty.THUMB_TOUCH, d, sleep_time=sleep_time)
        #return self.thumb_matrix
    
    def get_index_touch(self):
        """获取食指触觉传感数据"""
        if self.xB0[0] == 5:
            d = [23, 9, 1]
            sleep_time = 0.015
        else:
            d = [0xC6]
            sleep_time = 0.007
        self.send_command(FrameProperty.INDEX_TOUCH, d, sleep_time=sleep_time)
        #return self.xB2
    
    def get_middle_touch(self):
        """获取中指触觉传感数据"""
        if self.xB0[0] == 5:
            d = [23, 9, 1]
            sleep_time = 0.015
        else:
            d = [0xC6]
            sleep_time = 0.007
        self.send_command(FrameProperty.MIDDLE_TOUCH, d, sleep_time=sleep_time)
        #33333return self.xB3
    
    def get_ring_touch(self):
        """获取无名指触觉传感数据"""
        if self.xB0[0] == 5:
            d = [23, 9, 1]
            sleep_time = 0.015
        else:
            d = [0xC6]
            sleep_time = 0.007
        self.send_command(FrameProperty.RING_TOUCH, d, sleep_time=sleep_time)
        #return self.xB4
    
    def get_little_touch(self):
        """获取小拇指触觉传感数据"""
        if self.xB0[0] == 5:
            d = [23, 9, 1]
            sleep_time = 0.015
        else:
            d = [0xC6]
            sleep_time = 0.007
        self.send_command(FrameProperty.LITTLE_TOUCH, d, sleep_time=sleep_time)
        #return self.xB5
    
    def get_palm_touch(self):
        """获取手掌触觉传感数据"""
        if self.xB0[0] == 5:
            d = [28, 20, 1]
            sleep_time = 0.035
        else:
            d = [0xC6]
            sleep_time = 0.007
        self.send_command(FrameProperty.PALM_TOUCH, d, sleep_time=sleep_time)
        #return self.xB6

    # 查询指令方法
    def get_uid(self):
        """获取设备唯一标识码"""
        self.send_command(FrameProperty.HAND_UID_GET, [])
        return self.xC0
    
    def get_hardware_version(self):
        """获取硬件版本"""
        self.send_command(FrameProperty.HAND_HARDWARE_VERSION_GET, [])
        return self.xC1
    
    def get_software_version(self):
        """获取软件版本"""
        self.send_command(FrameProperty.HAND_SOFTWARE_VERSION_GET, [])
        return self.xC2
    
    def get_comm_id(self):
        """获取设备通信ID"""
        self.send_command(FrameProperty.HAND_COMM_ID_GET, [])
        return self.xC3
    
    def get_struct_version(self):
        """获取结构版本号"""
        self.send_command(FrameProperty.HAND_STRUCT_VERSION_GET, [])
        return self.xC4

    # 出厂指令方法
    def erase_position_calibration(self):
        """擦除位置校准值"""
        self.send_command(FrameProperty.HOST_CMD_HAND_ERASE_POS_CALI, [])
    
    def set_comm_id(self, new_id):
        """设置通信ID"""
        self.send_command(FrameProperty.HAND_COMM_ID_SET, [new_id])
    
    def set_uid(self, uid_data):
        """设置唯一标识码（内部出厂使用）"""
        self.send_command(FrameProperty.HAND_UID_SET, uid_data)

    # 辅助方法
    def slice_list(self, input_list, slice_size):
        """将列表按指定大小切片"""
        return [input_list[i:i + slice_size] for i in range(0, len(input_list), slice_size)]
    # -----------------------------------------------------
    # API指令区域
    #------------------------------------------------------
    def set_joint_positions(self, joint_ranges):
        """API接口:设置手指所有关节位置"""
        j = self.cmd_range_to_joint_range(cmd_list=joint_ranges)
        self.set_thumb_positions(j[0])
        self.set_index_positions(j[1])
        self.set_middle_positions(j[2])
        self.set_ring_positions(j[3])
        self.set_little_positions(j[4])

    def set_speed(self, speed=[250] * 5):
        """API接口:设置手指速度"""
        self.set_thumb_speed(speed_values=[speed[0]] * 6)
        self.set_index_speed(speed_values=[speed[1]] * 6)
        self.set_middle_speed(speed_values=[speed[2]] * 6)
        self.set_ring_speed(speed_values=[speed[3]] * 6)
        self.set_little_speed(speed_values=[speed[4]] * 6)

    def set_torque(self, torque=[250] * 5):
        """API接口:设置手指最大扭矩"""
        self.set_thumb_torque(torque_values=[torque[0]] * 6)
        self.set_index_torque(torque_values=[torque[1]] * 6)
        self.set_middle_torque(torque_values=[torque[2]] * 6)
        self.set_ring_torque(torque_values=[torque[3]] * 6)
        self.set_little_torque(torque_values=[torque[4]] * 6)


    def get_version(self):
        """API接口:获取手指嵌入式版本信息"""
        return self.get_software_version()
    
    def get_current_status(self):
        """API接口:获取手指当前状态"""
        self.get_thumb_positions()
        self.get_index_positions()
        self.get_middle_positions()
        self.get_ring_positions()
        self.get_little_positions()
        time.sleep(0.002)
        s = [self.x41, self.x42, self.x43, self.x44, self.x45]
        cmd_state = self.joint_state_to_cmd_state(state=s)
        return cmd_state

    def get_current_pub_status(self):
        """API接口:获取手指当前状态"""
        self.get_current_status()

    def get_speed(self):
        """API接口:获取手指速度"""
        self.get_thumb_speed()
        self.get_index_speed()
        self.get_middle_speed()
        self.get_ring_speed()
        self.get_little_speed()
        time.sleep(0.002)

        joint_speed = [self.x49, self.x4A, self.x4B, self.x4C, self.x4D]
        state_speed = self.joint_state_to_cmd_state(state=joint_speed)
        return state_speed

    def get_touch_type(self):
        """API接口:获取手指触觉传感器类型"""
        self.send_command(0xb0,[],sleep_time=0.03)
        self.send_command(0xb1,[],sleep_time=0.03)
        t = []
        for i in range(3):
            t = self.xB1
            time.sleep(0.01)
        if len(t) == 2:
            return 2
        else:
            self.send_command(0x20,[],sleep_time=0.03)
            time.sleep(0.01)
            if self.normal_force[0] == -1:
                return -1
            else:
                return 1
    
    def get_matrix_touch(self):
        """API接口:获取手指触摸传感器数据"""
        self.get_thumb_touch()
        self.get_index_touch()
        self.get_middle_touch()
        self.get_ring_touch()
        self.get_little_touch()
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix

    def get_matrix_touch_v2(self):
        """API接口:获取手指触摸传感器数据"""
        return self.get_matrix_touch()

    def get_thumb_matrix_touch(self,sleep_time=0):
        """API接口:获取[大拇指]指触摸传感器数据"""
        self.get_thumb_touch()
        if self.xB0[0] == 5:
            data = self.thumb_matrix_palm
        else:
            data = self.thumb_matrix
        return data
    
    def get_index_matrix_touch(self,sleep_time=0):
        """API接口:获取[食指]指触摸传感器数据"""
        self.get_index_touch()
        if self.xB0[0] == 5:
            data = self.index_matrix_palm
        else:
            data = self.index_matrix
        return data
    
    def get_middle_matrix_touch(self,sleep_time=0):
        """API接口:获取[中指]指触摸传感器数据"""
        self.get_middle_touch()
        if self.xB0[0] == 5:
            data = self.middle_matrix_palm
        else:
            data = self.middle_matrix
        return data
    
    def get_ring_matrix_touch(self,sleep_time=0):
        """API接口:获取[无名指]指触摸传感器数据"""
        self.get_ring_touch()
        if self.xB0[0] == 5:
            data = self.ring_matrix_palm
        else:
            data = self.ring_matrix
        return data
    
    def get_little_matrix_touch(self,sleep_time=0):
        """API接口:获取[小指]指触摸传感器数据"""
        self.get_little_touch()
        if self.xB0[0] == 5:
            data = self.little_matrix_palm
        else:
            data = self.little_matrix
        return data
    
    def get_palm_matrix_touch(self,sleep_time=0):
        """API接口:获取[小指]指触摸传感器数据"""
        self.get_palm_touch()
        if self.xB0[0] == 5:
            data = self.palm_matrix_palm
        else:
            data = self.palm_matrix
        return data

    def get_torque(self):
        """API接口:获取手指最大扭矩"""
        self.get_thumb_torque()
        self.get_index_torque()
        self.get_middle_torque()
        self.get_ring_torque()
        self.get_little_torque()
        time.sleep(0.003)
        t = [self.x51, self.x52, self.x53, self.x54, self.x55]
        cmd_torque = self.joint_state_to_cmd_state(state=t)
        return cmd_torque

    def get_current(self):
        """API接口:获取手指电流"""
        return [-1] * 20

    def get_temperature(self):
        """API接口:获取手指温度"""
        joint_temperature = [self.get_thumb_temperature(), self.get_index_temperature(), self.get_middle_temperature(), self.get_ring_temperature(), self.get_little_temperature()]
        cmd_temperature = self.joint_state_to_cmd_state(state=joint_temperature)
        return cmd_temperature


    def get_fault(self):
        """API接口:获取手指故障代码"""
        joint_fault = [self.get_thumb_fault(), self.get_index_fault(), self.get_middle_fault(), self.get_ring_fault(), self.get_little_fault()]
        cmd_fault = self.joint_state_to_cmd_state(state=joint_fault)
        return cmd_fault

    def clear_faults(self):
        """API接口:清除手指故障代码"""
        self.clear_finger_faults(finger_mask=[1, 1, 1, 1, 1])

    def cmd_range_to_joint_range(self,cmd_list):
        """根据手指映射关系，将手指控制命令列表转换为手指分组数据形式"""
        # 定义手指映射规则
        finger_mapping = {
            '拇指': [10, 5, 0, 11, 12, 15],
            '食指': [6, 11, 1, 13, 14, 16],
            '中指': [7, 12, 2, 13, 14, 17],
            '无名指': [8, 13, 3, 14, 15, 18],
            '小指': [9, 14, 4, 15, 16, 19]
        }
        
        result = []
        
        for finger, indices in finger_mapping.items():
            finger_data = [cmd_list[i] for i in indices]
            result.append(finger_data)
        
        return result
    

    def joint_state_to_cmd_state(self, state):
        """
        将关节状态转换为命令状态
        :param state: list2 格式的数据，5×6 的二维列表
        :return: list1 格式的 20 维列表
        """
        # 初始化结果列表，20个位置，预留位默认为0
        result = [0] * 20
        
        # list1 索引映射:
        # 0:拇指根部, 1:食指根部, 2:中指根部, 3:无名指根部, 4:小指根部
        # 5:拇指侧摆, 6:食指侧摆, 7:中指侧摆, 8:无名指侧摆, 9:小指侧摆
        # 10:拇指横摆, 11-14:预留, 15:拇指尖部, 16:食指末端, 17:中指末端, 18:无名指末端, 19:小指末端
        
        # list2 每行结构: [侧摆/横摆, 0, 根部, 0, 0, 末端/尖部]
        # 拇指行: [横摆, 侧摆, 根部, 0, 0, 尖部] — 注意拇指特殊，第1列是横摆，第2列是侧摆
        # 其他指: [侧摆, 0, 根部, 0, 0, 末端]
        
        # 拇指 (第0行) — 特殊处理
        result[10] = state[0][0]   # 拇指横摆
        result[5] = state[0][1]    # 拇指侧摆
        result[0] = state[0][2]    # 拇指根部
        result[15] = state[0][5]   # 拇指尖部
        
        # 食指 (第1行)
        result[6] = state[1][0]    # 食指侧摆
        result[1] = state[1][2]    # 食指根部
        result[16] = state[1][5]   # 食指末端
        
        # 中指 (第2行)
        result[7] = state[2][0]    # 中指侧摆
        result[2] = state[2][2]    # 中指根部
        result[17] = state[2][5]   # 中指末端
        
        # 无名指 (第3行)
        result[8] = state[3][0]    # 无名指侧摆
        result[3] = state[3][2]    # 无名指根部
        result[18] = state[3][5]   # 无名指末端
        
        # 小指 (第4行)
        result[9] = state[4][0]    # 小指侧摆
        result[4] = state[4][2]    # 小指根部
        result[19] = state[4][5]   # 小指末端
        
        # 预留位 11-14 保持为 0
        
        return result


    def _list_d_value(self, list1, list2):
        """检查两个列表的值是否有显著差异"""
        if list1 is None:
            return True
        for a, b in zip(list1, list2):
            if abs(b - a) > 2:
                return True
        return False

    def close_can_interface(self):
        """关闭CAN接口"""
        if self.bus:
            self.bus.shutdown()
            self.running = False
    
    def get_serial_number(self):
        try:
            self.send_command(0xC0,[],sleep_time=0.005)
            # 1. 使用 bytes() 函数将整数列表转换为字节对象
            #    bytes() 接收一个由 0-255 之间的整数组成的列表。
            byte_data = bytes(self.serial_number)
            # 2. 使用 .decode() 方法将字节对象解码为 ASCII 字符串
            result_string = byte_data.decode('ascii')
            if result_string == "":
                return "-1"
            else:
                # print(f"原始 ASCII 码列表: {self.serial_number}")
                # print(f"解码后的字符串: {result_string}")
                return result_string
        except:
            return "-1"
    def get_finger_order(self):
        return ["Thumb Base", "Index Finger Base", "Middle Finger Base", "Ring Finger Base", "Pinky Finger Base", "Thumb Abduction", "Index Finger Abduction", "Middle Finger Abduction", "Ring Finger Abduction", "Pinky Finger Abduction", "Thumb Horizontal Abduction", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Tip", "Index Finger Tip", "Middle Finger Tip", "Ring Finger Tip", "Pinky Finger Tip"]
