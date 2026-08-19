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


# ---------------------------------------------------------------------------
# 显示器缩放适配：DPI(像素密度) / 逻辑分辨率 / 窗口尺寸 三个维度
# ---------------------------------------------------------------------------
BASE_DPI = 96.0             # 设计基准 DPI
BASE_SCREEN_W = 1920.0      # 设计基准逻辑分辨率
BASE_SCREEN_H = 1080.0
BASE_WINDOW_W = 800.0       # 设计基准窗口尺寸(逻辑像素)
BASE_WINDOW_H = 400.0

# 基准字号(pt)与线宽，实际显示值 = 基准值 * 缩放系数
BASE_FONT_SIZES = {
    'title': 12.0,
    'label': 10.0,
    'tick': 9.0,
    'legend': 9.0,
}
BASE_LINE_WIDTH = 1.5


def _current_screen(widget=None):
    """获取窗口当前所在的显示器，取不到时回退到主显示器。"""
    app = QtWidgets.QApplication.instance()
    if app is None:
        return None
    if widget is not None:
        handle = widget.windowHandle()
        if handle is not None and handle.screen() is not None:
            return handle.screen()
        # Qt >= 5.10 才有 screenAt
        if hasattr(QtWidgets.QApplication, 'screenAt'):
            screen = QtWidgets.QApplication.screenAt(widget.geometry().center())
            if screen is not None:
                return screen
    return QtWidgets.QApplication.primaryScreen()


def screen_dpi(widget=None) -> float:
    """当前显示器的逻辑 DPI，用于设置 matplotlib 图形 DPI。

    matplotlib 的字号、线宽都以 pt 为单位，图形 DPI 跟随显示器后，
    文字与线条在高分屏上才会等比放大而不是变小、发虚。
    """
    screen = _current_screen(widget)
    if screen is None:
        return BASE_DPI
    dpi = float(screen.logicalDotsPerInch())
    if dpi <= 0:
        return BASE_DPI
    # 限幅，避免异常 DPI 造成图形过大/过小
    return max(BASE_DPI * 0.75, min(dpi, BASE_DPI * 3.0))


def screen_size_scale(widget=None) -> float:
    """逻辑分辨率相对基准(1920x1080)的尺寸缩放系数。

    小屏(如 1366x768)返回 <1 让文字收缩以免挤占绘图区，
    大屏返回 >1 让文字不至于过小。DPI 缩放已由 screen_dpi 处理，
    此处只负责"屏幕物理可用空间"这一维度，两者不重复叠加。
    """
    screen = _current_screen(widget)
    if screen is None:
        return 1.0
    avail = screen.availableGeometry()
    scale = min(avail.width() / BASE_SCREEN_W, avail.height() / BASE_SCREEN_H)
    return max(0.75, min(scale, 1.6))


def scaled_window_geometry(index: int = 0, base_w: int = int(BASE_WINDOW_W),
                           base_h: int = int(BASE_WINDOW_H)):
    """根据当前显示器可用区域计算窗口几何尺寸，实现多分辨率缩放适配。

    大屏上按 screen_size_scale 放大首选尺寸(base_w x base_h)，
    小屏上按可用区域比例收缩，并按窗口序号做少量偏移避免完全重叠。
    """
    screen = QtWidgets.QApplication.primaryScreen()
    if screen is None:
        return 100 + index * 50, 100 + index * 50, base_w, base_h
    avail = screen.availableGeometry()
    scale = screen_size_scale()
    w = min(int(base_w * scale), int(avail.width() * 0.6))
    h = min(int(base_h * scale), int(avail.height() * 0.6))
    step = int(40 * scale)
    x = avail.left() + int(60 * scale) + index * step
    y = avail.top() + int(60 * scale) + index * step
    # 保证偏移后窗口仍在可用区域内
    x = min(x, avail.right() - w)
    y = min(y, avail.bottom() - h)
    return x, y, w, h


def apply_app_scaling(app) -> float:
    """按显示器缩放 Qt 默认字体与 matplotlib 全局样式，返回缩放系数。

    需在 QApplication 创建之后调用。
    """
    scale = screen_size_scale()
    font = app.font()
    pt = font.pointSizeF()
    if pt <= 0:
        pt = 9.0
    font.setPointSizeF(round(pt * scale, 1))
    app.setFont(font)

    dpi = screen_dpi()
    matplotlib.rcParams.update({
        'figure.dpi': dpi,
        'font.size': BASE_FONT_SIZES['tick'] * scale,
        'axes.titlesize': BASE_FONT_SIZES['title'] * scale,
        'axes.labelsize': BASE_FONT_SIZES['label'] * scale,
        'xtick.labelsize': BASE_FONT_SIZES['tick'] * scale,
        'ytick.labelsize': BASE_FONT_SIZES['tick'] * scale,
        'legend.fontsize': BASE_FONT_SIZES['legend'] * scale,
        'lines.linewidth': BASE_LINE_WIDTH * scale,
    })
    return scale


class ScaledPlotWindow(QtWidgets.QMainWindow):
    """带显示器缩放适配的绘图窗口基类。

    适配三件事：
      1. 窗口几何 —— 按显示器可用区域缩放(scaled_window_geometry)
      2. 图形 DPI —— 跟随所在显示器，HiDPI 下图形清晰且物理尺寸一致
      3. 文字/线宽 —— 随显示器分辨率与当前窗口大小自适应，缩小窗口时
         标题、刻度、图例同步变小，不会挤掉绘图区
    子类在 super().__init__() 之后即可使用 self.canvas / self.ax。
    """

    def __init__(self, window_index: int = 0):
        super().__init__()
        self._size_scale = screen_size_scale()
        self._screen_hooked = False
        self.setGeometry(*scaled_window_geometry(window_index))
        # 最小尺寸同样缩放，避免高分屏上窗口被缩到无法阅读
        self.setMinimumSize(int(360 * self._size_scale), int(240 * self._size_scale))

        # 图形设置：DPI 跟随显示器；使用 constrained 布局，随窗口/分辨率
        # 自动调整边距，避免标题、坐标轴标签和图例在不同缩放下被裁剪
        dpi = screen_dpi()
        self.canvas = FigureCanvasQTAgg(Figure(
            figsize=(BASE_WINDOW_W * self._size_scale / dpi,
                     BASE_WINDOW_H * self._size_scale / dpi),
            dpi=dpi,
            layout='constrained'))
        self.setCentralWidget(self.canvas)
        self.ax = self.canvas.figure.add_subplot(111)
        self.ax.grid(True)

    # ---- 缩放适配 ----------------------------------------------------
    def _text_scale(self) -> float:
        """显示器系数 x 窗口相对基准尺寸的系数(限幅)。"""
        w = max(self.canvas.width(), 1)
        h = max(self.canvas.height(), 1)
        rel = min(w / (BASE_WINDOW_W * self._size_scale),
                  h / (BASE_WINDOW_H * self._size_scale))
        rel = max(0.65, min(rel, 1.8))
        # 两个系数相乘后再次限幅：下限保证小屏小窗口时文字仍可读，
        # 上限避免大屏最大化时文字过大挤占绘图区
        return max(0.8, min(self._size_scale * rel, 2.0))

    def apply_text_scale(self):
        """把当前缩放系数应用到标题、坐标轴、刻度、图例和线宽。"""
        s = self._text_scale()
        self.ax.title.set_fontsize(BASE_FONT_SIZES['title'] * s)
        self.ax.xaxis.label.set_fontsize(BASE_FONT_SIZES['label'] * s)
        self.ax.yaxis.label.set_fontsize(BASE_FONT_SIZES['label'] * s)
        self.ax.tick_params(axis='both', labelsize=BASE_FONT_SIZES['tick'] * s,
                            width=0.8 * s, length=3.5 * s)
        for line in self.ax.get_lines():
            line.set_linewidth(BASE_LINE_WIDTH * s)
        for spine in self.ax.spines.values():
            spine.set_linewidth(0.8 * s)
        legend = self.ax.get_legend()
        if legend is not None:
            for text in legend.get_texts():
                text.set_fontsize(BASE_FONT_SIZES['legend'] * s)
            frame = legend.get_frame()
            if frame is not None:
                frame.set_linewidth(0.8 * s)
        self.canvas.draw_idle()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.apply_text_scale()

    def showEvent(self, event):
        super().showEvent(event)
        handle = self.windowHandle()
        if handle is not None and not self._screen_hooked:
            # 窗口被拖到另一台显示器时重新适配
            handle.screenChanged.connect(self._on_screen_changed)
            self._screen_hooked = True
        self.apply_text_scale()

    def _on_screen_changed(self, *_):
        """跨显示器移动后重算 DPI 与字号(混合分辨率多屏场景)。"""
        self._size_scale = screen_size_scale(self)
        self.canvas.figure.set_dpi(screen_dpi(self))
        self.setMinimumSize(int(360 * self._size_scale), int(240 * self._size_scale))
        self.apply_text_scale()


class ForceGroupWindow(ScaledPlotWindow):
    """专用力传感器组可视化窗口"""
    def __init__(self, group_id: int):
        super().__init__(window_index=group_id)
        self.setWindowTitle(f"Force Sensor Group {group_id+1}")
        self.ax.set_title(f'Force Group {group_id+1} (5 channels)')
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel('Force (N)')

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
        # 支持 125%/150% 等非整数缩放(Qt >= 5.14)，避免向下取整导致界面偏小
        if hasattr(QtCore.Qt, 'HighDpiScaleFactorRoundingPolicy'):
            QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(
                QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
        self.app = QtWidgets.QApplication(sys.argv)
        # 按当前显示器缩放 Qt 字体与 matplotlib 全局字号/DPI
        self.ui_scale = apply_app_scaling(self.app)
        self.get_logger().info(f"Display scale factor: {self.ui_scale:.2f}, dpi: {screen_dpi():.0f}")
        
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

class DataPlotWindow(ScaledPlotWindow):
    """通用数据绘图窗口"""
    def __init__(self, title: str, ylabel: str, channel_count: int, y_range: tuple):
        super().__init__()
        self.setWindowTitle(title)
        self.ax.set_title(title)
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel(ylabel)

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


