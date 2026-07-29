import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5 import QtWidgets, QtCore
import numpy as np
import json
import sys
from typing import Dict, List

from linker_hand_ros2_sdk.LinkerHand.utils.init_linker_hand import InitLinkerHand


def scaled_window_geometry(index: int = 0, base_w: int = 800, base_h: int = 400):
    """根据当前显示器可用区域计算窗口几何尺寸，实现多分辨率缩放适配。

    在大屏上使用首选尺寸(base_w x base_h)，在小屏上按可用区域比例收缩，
    并按窗口序号做少量偏移，避免多个窗口完全重叠。
    """
    screen = QtWidgets.QApplication.primaryScreen()
    if screen is None:
        return 100 + index * 50, 100 + index * 50, base_w, base_h
    avail = screen.availableGeometry()
    w = min(base_w, int(avail.width() * 0.6))
    h = min(base_h, int(avail.height() * 0.6))
    x = avail.left() + 60 + index * 40
    y = avail.top() + 60 + index * 40
    # 保证偏移后窗口仍在可用区域内
    x = min(x, avail.right() - w)
    y = min(y, avail.bottom() - h)
    return x, y, w, h


class ForceGroupWindow(QtWidgets.QMainWindow):
    """专用力传感器组可视化窗口"""
    def __init__(self, group_id: int):
        super().__init__()
        self.setWindowTitle(f"Force Sensor Group {group_id+1}")
        self.setGeometry(*scaled_window_geometry(group_id))
        
        # 图形设置：使用 constrained 布局，随窗口/分辨率自动调整边距，
        # 避免标题、坐标轴标签和图例在不同缩放下被裁剪
        self.canvas = FigureCanvasQTAgg(Figure(figsize=(8, 4), layout='constrained'))
        self.setCentralWidget(self.canvas)
        self.ax = self.canvas.figure.add_subplot(111)
        self.ax.set_title(f'Force Group {group_id+1} (5 channels)')
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel('Force (N)')
        self.ax.grid(True)
        
        # 数据存储
        self.buffer_size = 200
        self.x_data = np.arange(self.buffer_size)
        #self.channels = [f'Channel {i+1}' for i in range(5)]
        self.channels = ["thumb","index finger","middle finger","ring finger","little finger"]
        self.data = {name: np.full(self.buffer_size, np.nan) for name in self.channels}
        self.lines = {}
        self.data_ptr = 0
        
        # 颜色设置
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        
        # 创建曲线
        for i, name in enumerate(self.channels):
            self.lines[name], = self.ax.plot(
                self.x_data,
                self.data[name],
                color=colors[i],
                label=name,
                linewidth=1.5
            )
        
        self.ax.legend(loc='upper right')
        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(0, 300)  # 假设力传感器范围0-300N
        
        # 定时刷新
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # 20fps
    
    def add_data(self, new_data: List[float]):
        """添加新数据点"""
        self.data_ptr = (self.data_ptr + 1) % self.buffer_size
        for name, val in zip(self.channels, new_data):
            self.data[name][self.data_ptr] = float(val)
    
    def update_plot(self):
        """更新绘图"""
        # 更新曲线数据
        for name, line in self.lines.items():
            line.set_ydata(np.roll(self.data[name], -self.data_ptr))
        
        self.canvas.draw()

class HandMonitor(Node):
    def __init__(self):
        super().__init__('graphic_display')
        
        # 初始化Qt应用
        # 启用高分辨率(HiDPI)缩放适配，必须在创建 QApplication 之前设置
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
        self.app = QtWidgets.QApplication(sys.argv)
        
        # 窗口管理
        self.force_windows = {}  # 存储force组窗口 {group_id: window}
        self.temp_window = None  # 温度窗口
        self.torque_window = None  # 扭矩窗口
        self.hand_joint, self.hand_type = InitLinkerHand().current_hand()
        if self.hand_type == "left":
            self.topic = "/cb_left_hand_info"
        else:
            self.topic = "/cb_right_hand_info"
        #self.topic = "/cb_left_hand_info"
        # ROS2订阅
        self.subscription = self.create_subscription(
            String,
            self.topic,
            self.data_callback,
            10)
        
        # Qt事件处理定时器
        self.timer = self.create_timer(0.1, self.process_qt_events)
        
        self.get_logger().info("Hand monitor initialized")
    
    def data_callback(self, msg: String):
        """处理手部数据回调"""
        try:
            data = json.loads(msg.data)
            if self.hand_type == "left":
                tmp = "left_hand"
            else:
                tmp = "right_hand"
            if isinstance(data, dict) and tmp in data:
                hand_data = data[tmp]
                
                # 处理force数据 (每组force一个独立窗口)
                if 'force' in hand_data:
                    force_data = hand_data['force']
                    for group_id, group_values in enumerate(force_data):
                        if len(group_values) == 5:  # 每组应有5个值
                            if group_id not in self.force_windows:
                                self.force_windows[group_id] = ForceGroupWindow(group_id)
                                self.force_windows[group_id].show()
                            
                            # 跨线程安全更新
                            if QtCore.QThread.currentThread() == self.app.thread():
                                self.force_windows[group_id].add_data(group_values)
                            else:
                                QtCore.QMetaObject.invokeMethod(
                                    self.force_windows[group_id],
                                    'add_data',
                                    QtCore.Qt.QueuedConnection,
                                    QtCore.Q_ARG(list, group_values)
                                )
                
                # 处理温度数据 (单个窗口)
                if 'motor_temperature' in hand_data:
                    temp_data = hand_data['motor_temperature']
                    if len(temp_data) == 10:  # 应有10个温度值
                        if self.temp_window is None:
                            self.create_temp_window()
                        self.update_window_data(self.temp_window, temp_data)
                
                # 处理扭矩数据 (单个窗口)
                # if 'torque' in hand_data:
                #     torque_data = hand_data['torque']
                #     if len(torque_data) == 5:  # 应有5个扭矩值
                #         if self.torque_window is None:
                #             self.create_torque_window()
                #         self.update_window_data(self.torque_window, torque_data)
        
        except Exception as e:
            self.get_logger().error(f"Data processing error: {str(e)}")
    
    def create_temp_window(self):
        """创建温度窗口"""
        self.temp_window = DataPlotWindow(
            title="Motor Temperatures",
            ylabel="Temperature (°C)",
            channel_count=10,
            y_range=(20, 50)
        )
        self.temp_window.show()
    
    def create_torque_window(self):
        """创建扭矩窗口"""
        self.torque_window = DataPlotWindow(
            title="Joint Torque",
            ylabel="Torque (Nm)",
            channel_count=5,
            y_range=(-0.5, 0.5)
        )
        self.torque_window.show()
    
    def update_window_data(self, window, data):
        """通用窗口数据更新"""
        if QtCore.QThread.currentThread() == self.app.thread():
            window.add_data(data)
        else:
            QtCore.QMetaObject.invokeMethod(
                window,
                'add_data',
                QtCore.Qt.QueuedConnection,
                QtCore.Q_ARG(list, data)
            )
    
    def process_qt_events(self):
        """处理Qt事件循环"""
        self.app.processEvents()
        
        # 清理已关闭的窗口
        self.force_windows = {k: v for k, v in self.force_windows.items() if v.isVisible()}
        if self.temp_window and not self.temp_window.isVisible():
            self.temp_window = None
        if self.torque_window and not self.torque_window.isVisible():
            self.torque_window = None
    
    def run(self):
        """启动Qt应用"""
        self.app.exec_()
    
    def destroy_node(self):
        """清理资源"""
        for window in self.force_windows.values():
            window.close()
        if self.temp_window:
            self.temp_window.close()
        if self.torque_window:
            self.torque_window.close()
        super().destroy_node()

class DataPlotWindow(QtWidgets.QMainWindow):
    """通用数据绘图窗口"""
    def __init__(self, title: str, ylabel: str, channel_count: int, y_range: tuple):
        super().__init__()
        self.setWindowTitle(title)
        self.setGeometry(*scaled_window_geometry())
        
        # 图形设置：使用 constrained 布局，为外置图例预留空间，避免被裁剪
        self.canvas = FigureCanvasQTAgg(Figure(figsize=(8, 4), layout='constrained'))
        self.setCentralWidget(self.canvas)
        self.ax = self.canvas.figure.add_subplot(111)
        self.ax.set_title(title)
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel(ylabel)
        self.ax.grid(True)
        
        # 数据存储
        self.buffer_size = 200
        self.x_data = np.arange(self.buffer_size)
        self.channels = [f'Channel {i+1}' for i in range(channel_count)]
        #self.channels = ["thumb","index finger","middle finger","ring finger","little finger"]
        self.data = {name: np.full(self.buffer_size, np.nan) for name in self.channels}
        self.lines = {}
        self.data_ptr = 0
        
        # 创建曲线
        colors = matplotlib.colormaps['tab20'].colors
        for i, name in enumerate(self.channels):
            self.lines[name], = self.ax.plot(
                self.x_data,
                self.data[name],
                color=colors[i % len(colors)],
                label=name,
                linewidth=1.5
            )
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(*y_range)
        
        # 定时刷新
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)
    
    def add_data(self, new_data: List[float]):
        """添加新数据点"""
        self.data_ptr = (self.data_ptr + 1) % self.buffer_size
        for name, val in zip(self.channels, new_data):
            self.data[name][self.data_ptr] = float(val)
    
    def update_plot(self):
        """更新绘图"""
        for name, line in self.lines.items():
            line.set_ydata(np.roll(self.data[name], -self.data_ptr))
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    
    # 必须在主线程创建节点
    monitor = HandMonitor()
    
    # 启动Qt线程
    from threading import Thread
    qt_thread = Thread(target=monitor.run, daemon=True)
    qt_thread.start()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


