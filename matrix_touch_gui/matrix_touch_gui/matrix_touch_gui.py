
#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout)
from PyQt5.QtCore import QTimer, QObject, pyqtSignal

from .views.left_widget import LeftControlWidget
from .views.right_widget import RightMatrixWidget

class HandMatrixGui(Node, QObject):
    """带刷新频率控制的手掌矩阵界面"""
    update_signal = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'raw_data_gui_node')
        QObject.__init__(self)

        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L10')
        
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value

        # 刷新控制参数
        self.refresh_interval = 200  # 默认刷新间隔200ms
        self.data_cache = {}  # 用于缓存数据

        # 创建刷新定时器
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.process_cached_data)
        self.refresh_timer.start(self.refresh_interval)

        # 创建应用和窗口
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("ROS2手指点阵界面（红色渐变版）")
        self.window.setGeometry(100, 100, 900, 650)

        # 创建中心部件
        central_widget = QWidget()
        self.window.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # 加载左侧控制面板
        self.left_control = LeftControlWidget(self.hand_type)
        self.left_control.subscribe_clicked.connect(self.toggle_subscription)
        self.left_control.refresh_interval_changed.connect(self.set_refresh_interval)
        
        # 加载右侧矩阵显示
        self.right_matrix = RightMatrixWidget()

        # 添加到主布局
        main_layout.addWidget(self.left_control, 2)
        main_layout.addWidget(self.right_matrix, 1)

        # 初始化变量
        self.subscription = None
        self.is_subscribed = False

        # 显示窗口
        self.window.show()
        self.set_refresh_interval(interval=200)

        # 添加初始消息
        self.add_message("红色渐变版界面已启动")
        self.add_message("当前刷新频率: {}ms".format(self.refresh_interval))
        self.add_message("请选择话题并点击'订阅/取消'按钮")

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
                self.add_message(f"已更新{display_name}矩阵数据：{matrix_data}")
            
            # 清空缓存
            self.data_cache.clear()

    def set_refresh_interval(self, interval=200):
        if interval < 200:
            self.add_message("刷新间隔不能低于200ms")
            return
        self.refresh_interval = interval
        self.refresh_timer.setInterval(interval)
        self.add_message(f"刷新频率已调整为{interval}ms")

    def add_message(self, text):
        """添加消息到终端"""
        self.left_control.add_terminal_message(text)

    def run(self):
        sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    try:
        gui_node = HandMatrixGui()
        
        # 在单独线程中运行ROS2 spin
        import threading
        spin_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
        spin_thread.start()
        
        gui_node.run()
        rclpy.shutdown()
        spin_thread.join()
    except Exception as e:
        print(f"应用启动失败: {str(e)}")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
