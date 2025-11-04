#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading
import time

from .views.left_widget import LeftControlWidget
from .views.right_widget import RightMatrixWidget

class HandMatrixGui(Node):
    """带刷新频率控制的手掌矩阵界面 - Tkinter版本"""
    
    def __init__(self):
        Node.__init__(self, 'raw_data_gui_node')

        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L10')
        
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value

        # 刷新控制参数
        self.refresh_interval = 200  # 默认刷新间隔200ms
        self.data_cache = {}  # 用于缓存数据

        # 创建Tkinter应用和窗口
        self.root = tk.Tk()
        self.root.title("ROS2手指点阵界面（红色渐变版）")
        self.root.geometry('1000x950')

        # 创建中心部件
        central_widget = ttk.Frame(self.root)
        central_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建水平布局
        main_frame = ttk.Frame(central_widget)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 加载左侧控制面板 - 权重2
        self.left_control = LeftControlWidget(main_frame, self.hand_type)
        self.left_control.set_subscribe_callback(self.toggle_subscription)
        self.left_control.set_refresh_callback(self.set_refresh_interval)
        self.left_control.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 加载右侧矩阵显示 - 权重1
        self.right_matrix = RightMatrixWidget(main_frame)
        self.right_matrix.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # 初始化变量
        self.subscription = None
        self.is_subscribed = False
        
        # 创建刷新定时器
        self.refresh_timer_id = None
        self.start_refresh_timer()

        # 添加初始消息
        self.add_message("红色渐变版界面已启动")
        self.add_message(f"当前刷新频率: {self.refresh_interval}ms")
        self.add_message("请选择话题并点击'订阅/取消'按钮")

        # 初始化显示默认点阵
        self.initialize_default_matrices()

    def initialize_default_matrices(self):
        """初始化显示默认点阵（所有点为灰色）"""
        # 为每个手指创建空的默认数据（全0）
        default_data = [[0] * 6 for _ in range(12)]  # 12x6的全0矩阵
        flattened_data = [item for sublist in default_data for item in sublist]
        
        finger_names = ["thumb_matrix", "index_matrix", "middle_matrix", "ring_matrix", "little_matrix"]
        for finger_name in finger_names:
            self.right_matrix.update_matrix_data(finger_name, flattened_data)

    def toggle_subscription(self):
        if self.is_subscribed:
            self.unsubscribe()
        else:
            self.subscribe()

    def subscribe(self):
        topic = self.left_control.get_selected_topic()
        self.add_message(f"正在订阅话题: {topic}")
        self.subscription = self.create_subscription(
            String,
            topic,
            self.topic_callback,
            10
        )
        self.is_subscribed = True
        self.left_control.update_status(f"状态: 已订阅 {topic} | 红色渐变显示", True)
        self.add_message(f"已订阅话题: {topic}")

    def unsubscribe(self):
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.is_subscribed = False
            self.left_control.update_status("状态: 未订阅 | 红色渐变显示", False)
            self.add_message("已取消订阅")
            # 取消订阅后恢复默认灰色点阵
            self.initialize_default_matrices()

    def topic_callback(self, msg):
        try:
            data = json.loads(msg.data)
            for finger_name, matrix_data in data.items():
                # 缓存数据，由定时器统一更新
                self.data_cache[finger_name] = matrix_data
        except json.JSONDecodeError as e:
            self.add_message(f"JSON解析错误: {str(e)}")
        except Exception as e:
            self.add_message(f"数据处理错误: {str(e)}")

    def process_cached_data(self):
        """定时处理缓存的数据并更新界面"""
        if self.data_cache and self.is_subscribed:
            for finger_name, matrix_data in self.data_cache.items():
                # 更新点阵显示
                self.right_matrix.update_matrix_data(finger_name, matrix_data)
                
                # 更新终端显示
                display_name = {
                    "thumb_matrix": "拇指",
                    "index_matrix": "食指",
                    "middle_matrix": "中指",
                    "ring_matrix": "无名指",
                    "little_matrix": "小指"
                }.get(finger_name, finger_name)
                self.add_message(f"已更新{display_name}矩阵数据:{matrix_data}")
            
            # 清空缓存
            self.data_cache.clear()
            
        # 继续定时器
        self.start_refresh_timer()

    def start_refresh_timer(self):
        """启动刷新定时器"""
        if self.refresh_timer_id:
            self.root.after_cancel(self.refresh_timer_id)
        self.refresh_timer_id = self.root.after(self.refresh_interval, self.process_cached_data)

    def set_refresh_interval(self, interval=200):
        if interval < 200:
            self.add_message("刷新间隔不能低于200ms")
            return
        self.refresh_interval = interval
        self.add_message(f"刷新频率已调整为{interval}ms")
        self.start_refresh_timer()

    def add_message(self, text):
        """添加消息到终端"""
        self.left_control.add_terminal_message(text)

    def run(self):
        """运行GUI"""
        try:
            # 启动ROS2自旋线程
            spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
            spin_thread.start()
            
            self.root.mainloop()
        finally:
            # 清理定时器
            if self.refresh_timer_id:
                self.root.after_cancel(self.refresh_timer_id)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        gui_node = HandMatrixGui()
        gui_node.run()
    except Exception as e:
        print(f"应用启动失败: {str(e)}")
        rclpy.shutdown()

if __name__ == "__main__":
    main()