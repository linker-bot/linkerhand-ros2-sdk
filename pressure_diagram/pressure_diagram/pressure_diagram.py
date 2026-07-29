#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import signal
import sys
import json
import numpy as np
from std_msgs.msg import String
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

pg.setConfigOption('imageAxisOrder', 'row-major')
pg.setConfigOption('useOpenGL', False)  

class PressureDiagram(Node, QtWidgets.QMainWindow):
    def __init__(self):
        # 先调用 Node 的 __init__，避免 super() 歧义
        Node.__init__(self, 'pressure_diagram')
        QtWidgets.QMainWindow.__init__(self)

        # 键名与ROS数据一致
        self.fingers_matrix = ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']
        self.fingers_mass = ['thumb_mass', 'index_mass', 'middle_mass', 'ring_mass', 'little_mass']

        # 掌心(palm)为可选通道：数据不存在，或持续无效(-1)时不显示
        self.palm_mass_key = 'palm_mass'
        self.palm_matrix_key = 'palm_matrix'
        self.palm_hide_frames = 5  # 连续多少帧无效则隐藏

        self.finger_colors = [
            (255, 75, 75),    # Thumb - Red
            (50, 205, 50),    # Index - Green
            (65, 105, 255),   # Middle - Blue
            (255, 215, 0),    # Ring - Gold
            (218, 112, 214)   # Pinky - Purple
        ]
        self.palm_color = (34, 211, 238)  # Palm - Cyan

        # 波形图数据（含 palm）
        self.wave_data = {f: np.zeros(100) for f in self.fingers_mass}
        self.wave_data[self.palm_mass_key] = np.zeros(100)
        self.current_wave_values = {f: 0.0 for f in self.fingers_mass}
        self.current_wave_values[self.palm_mass_key] = 0.0

        # palm 波形显示状态
        self.palm_wave_present = False      # 最近一帧是否包含 palm_mass
        self.palm_wave_neg_streak = 0       # palm_mass 连续为 -1 的帧数
        self.palm_wave_plot = None          # palm 波形子图（按需创建）

        # 热力图数据 - 12行×6列 (高大于宽)
        self.matrix_data = {f: np.zeros((12, 6)) for f in self.fingers_matrix}
        self.matrix_shapes = {f: (12, 6) for f in self.fingers_matrix}  # (rows, cols)
        self.data_received = {f: False for f in self.fingers_matrix}

        # palm 热力图数据与显示状态（尺寸不固定）
        self.matrix_data[self.palm_matrix_key] = np.zeros((1, 1))
        self.matrix_shapes[self.palm_matrix_key] = (1, 1)
        self.data_received[self.palm_matrix_key] = False
        self.palm_matrix_present = False    # 最近一帧是否包含 palm_matrix
        self.palm_matrix_neg_streak = 0     # palm_matrix 全为 -1 的连续帧数
        self.palm_heat_container = None     # palm 热力图容器（预创建，按需显隐）

        self.wave_sub = None
        self.matrix_sub = None

        # ROS2 节流计时器
        self.last_matrix_log_time = {f: None for f in self.fingers_matrix}
        self.last_matrix_log_time[self.palm_matrix_key] = None

        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.qos_profile = qos_profile

        self.init_ui()
        self.setup_ros()

    def init_ui(self):
        self.setWindowTitle("Robotic Hand Sensor Fusion Interface")
        self.resize(1600, 900)
        self.setStyleSheet("background-color: #0f172a;")
        
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # ============== 左侧：波形图 ==============
        left_widget = self.create_waveform_panel()
        main_layout.addWidget(left_widget, stretch=1)
        
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.VLine)
        line.setStyleSheet("QFrame { background-color: #334155; max-width: 2px; }")
        main_layout.addWidget(line)
        
        # ============== 右侧：热力图 ==============
        right_widget = self.create_heatmap_panel()
        main_layout.addWidget(right_widget, stretch=1)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33)

    def create_waveform_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setSpacing(8)
        
        wave_ctrl = QtWidgets.QFrame()
        wave_ctrl.setStyleSheet("""QFrame { background-color: #1e293b; border: 2px solid #3b82f6; border-radius: 4px; }""")
        wave_layout = QtWidgets.QHBoxLayout(wave_ctrl)
        wave_layout.setContentsMargins(12, 6, 12, 6)
        
        title = QtWidgets.QLabel("WAVEFORM MONITOR (30Hz)")
        title.setStyleSheet("color: #60a5fa; font-weight: bold; font-size: 13px;")
        
        self.wave_combo = QtWidgets.QComboBox()
        self.wave_combo.addItems([
            "/cb_left_hand_matrix_touch_mass",
            "/cb_right_hand_matrix_touch_mass"
        ])
        self.wave_combo.setStyleSheet("""
            QComboBox { color: white; background-color: #334155; border: 1px solid #475569; border-radius: 4px; padding: 4px 8px; min-width: 260px; }
            QComboBox QAbstractItemView { background-color: #1e293b; color: white; selection-background-color: #3b82f6; }
        """)
        self.wave_combo.currentTextChanged.connect(self.switch_wave_topic)
        
        wave_layout.addWidget(title)
        wave_layout.addWidget(self.wave_combo)
        wave_layout.addStretch()
        layout.addWidget(wave_ctrl)
        
        self.wave_widget = pg.GraphicsLayoutWidget()
        self.wave_widget.setBackground('#0f172a')
        layout.addWidget(self.wave_widget, stretch=1)
        
        self.wave_plots = {}
        self.wave_curves = {}

        for i, finger in enumerate(self.fingers_mass):
            # 默认最后一个指头显示 Time 轴（若 palm 出现，会自动移交给 palm）
            self._add_wave_plot(i, finger, self.finger_colors[i],
                                show_time=(i == len(self.fingers_mass) - 1))

        return panel

    def _set_wave_time_axis(self, plot, show_time):
        """控制某个波形子图底部 Time 轴的显示（仅最底部子图显示）"""
        axis = plot.getAxis('bottom')
        if show_time:
            axis.setStyle(showValues=True)
            axis.setLabel('Time', color='#64748b')
            axis.showLabel(True)
        else:
            axis.setStyle(showValues=False)
            axis.showLabel(False)

    def _add_wave_plot(self, row, finger, color, show_time):
        """创建一个波形子图并登记曲线/数值文本，供手指与 palm 复用"""
        p = self.wave_widget.addPlot(row=row, col=0)
        p.setMenuEnabled(False)
        p.setMouseEnabled(x=False, y=False)
        p.setYRange(0, 4500)
        p.setXRange(0, 100)
        p.showGrid(x=True, y=True, alpha=0.3)
        p.getAxis('left').setTextPen('#64748b')
        p.getAxis('bottom').setTextPen('#64748b')

        qcolor = QtGui.QColor(*color)
        display_name = finger.replace('_mass', '').upper()
        p.setTitle(f"[ {display_name} ]", color=qcolor, size='11pt')
        self._set_wave_time_axis(p, show_time)

        pen = pg.mkPen(color=color, width=2.5)
        curve = p.plot(pen=pen)
        self.wave_curves[finger] = curve

        text = pg.TextItem(text="0", color=(255, 255, 255), anchor=(1, 0.5))
        text.setFont(QtGui.QFont("Arial", 10, QtGui.QFont.Bold))
        p.addItem(text)
        self.wave_curves[finger + '_text'] = text

        self.wave_plots[finger] = p
        return p

    def _create_palm_wave_plot(self):
        """按需创建 palm 波形子图（置于最底部，并接管 Time 轴）"""
        if self.palm_wave_plot is not None:
            return
        last_finger = self.fingers_mass[-1]
        if last_finger in self.wave_plots:
            self._set_wave_time_axis(self.wave_plots[last_finger], False)
        row = len(self.fingers_mass)
        self.palm_wave_plot = self._add_wave_plot(
            row, self.palm_mass_key, self.palm_color, show_time=True)

    def _remove_palm_wave_plot(self):
        """移除 palm 波形子图并把 Time 轴交还给最后一个指头"""
        if self.palm_wave_plot is None:
            return
        self.wave_widget.removeItem(self.palm_wave_plot)
        self.palm_wave_plot = None
        self.wave_curves.pop(self.palm_mass_key, None)
        self.wave_curves.pop(self.palm_mass_key + '_text', None)
        self.wave_plots.pop(self.palm_mass_key, None)
        last_finger = self.fingers_mass[-1]
        if last_finger in self.wave_plots:
            self._set_wave_time_axis(self.wave_plots[last_finger], True)

    def create_heatmap_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setSpacing(8)
        
        heat_ctrl = QtWidgets.QFrame()
        heat_ctrl.setStyleSheet("""QFrame { background-color: #1e293b; border: 2px solid #f43f5e; border-radius: 4px; }""")
        heat_layout = QtWidgets.QHBoxLayout(heat_ctrl)
        heat_layout.setContentsMargins(12, 6, 12, 6)
        
        title = QtWidgets.QLabel("PRESSURE MATRIX (12×6)")
        title.setStyleSheet("color: #f43f5e; font-weight: bold; font-size: 13px;")
        
        # 单位标签
        unit_label = QtWidgets.QLabel("Max Value (g)")
        unit_label.setStyleSheet("color: #94a3b8; font-size: 11px; border: none; background: transparent;")
        heat_layout.addWidget(unit_label)
        heat_layout.addSpacing(10)
        
        self.matrix_combo = QtWidgets.QComboBox()
        self.matrix_combo.addItems([
            "/cb_left_hand_matrix_touch",
            "/cb_right_hand_matrix_touch"
        ])
        self.matrix_combo.setStyleSheet("""
            QComboBox { color: white; background-color: #334155; border: 1px solid #475569; border-radius: 4px; padding: 4px 8px; min-width: 260px; }
            QComboBox QAbstractItemView { background-color: #1e293b; color: white; selection-background-color: #f43f5e; }
        """)
        self.matrix_combo.currentTextChanged.connect(self.switch_matrix_topic)
        
        heat_layout.addWidget(title)
        heat_layout.addWidget(self.matrix_combo)
        heat_layout.addStretch()
        layout.addWidget(heat_ctrl)
        
        heat_container = QtWidgets.QWidget()
        heat_grid = QtWidgets.QGridLayout(heat_container)
        heat_grid.setSpacing(10)
        heat_grid.setContentsMargins(5, 5, 5, 5)
        
        self.matrix_plots = {}
        self.matrix_images = {}
        self.matrix_peaks = {}
        
        positions = [
            ('thumb_matrix', 0, 0), ('index_matrix', 0, 1), ('middle_matrix', 0, 2),
            ('ring_matrix', 1, 0), ('little_matrix', 1, 1)
        ]
        
        self.colormap = pg.colormap.get('plasma')

        for idx, (finger, row, col) in enumerate(positions):
            container = self._create_matrix_view(
                finger, self.finger_colors[idx],
                finger.replace('_matrix', '').upper(),
                sizes=(240, 280, 120, 160)
            )
            heat_grid.addWidget(container, row, col)
        
        # 颜色条
        legend_widget = QtWidgets.QWidget()
        legend_layout = QtWidgets.QVBoxLayout(legend_widget)
        legend_layout.setAlignment(QtCore.Qt.AlignCenter)
        
        lbl_max = QtWidgets.QLabel("MAX")
        lbl_max.setStyleSheet("color: #fbbf24; font-size: 9px;")
        lbl_max.setAlignment(QtCore.Qt.AlignCenter)
        
        gradient = pg.GradientWidget(orientation='right')
        gradient.setMaximumWidth(25)
        gradient.setMaximumHeight(180)
        gradient.setColorMap(self.colormap)
        
        lbl_min = QtWidgets.QLabel("0")
        lbl_min.setStyleSheet("color: #3b82f6; font-size: 9px;")
        lbl_min.setAlignment(QtCore.Qt.AlignCenter)
        
        legend_layout.addWidget(lbl_max)
        legend_layout.addWidget(gradient, stretch=1)
        legend_layout.addWidget(lbl_min)
        
        heat_grid.addWidget(legend_widget, 1, 2)

        # palm 热力图（尺寸不固定），单独占一行并横跨三列；默认隐藏，
        # 只有收到有效 palm_matrix 数据时才显示
        self.palm_heat_container = self._create_matrix_view(
            self.palm_matrix_key, self.palm_color, 'PALM',
            sizes=(520, 340, 200, 160)
        )
        self.palm_heat_container.setVisible(False)
        heat_grid.addWidget(self.palm_heat_container, 2, 0, 1, 3)

        self.heat_grid = heat_grid

        layout.addWidget(heat_container, stretch=1)
        return panel

    def _create_matrix_view(self, finger, color, name, sizes):
        """创建一个热力图子视图并登记，供手指与 palm 复用。

        sizes = (max_w, max_h, min_w, min_h)
        """
        max_w, max_h, min_w, min_h = sizes
        container = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(container)
        vbox.setSpacing(2)
        vbox.setContentsMargins(0, 0, 0, 0)

        name_label = QtWidgets.QLabel(name)
        name_label.setAlignment(QtCore.Qt.AlignCenter)
        color_hex = '#{:02x}{:02x}{:02x}'.format(*color)
        name_label.setStyleSheet(f"color: {color_hex}; font-weight: bold; font-size: 12px;")
        vbox.addWidget(name_label)

        view = pg.PlotWidget()
        view.setMenuEnabled(False)
        view.setMouseEnabled(x=False, y=False)
        view.setAspectLocked(True, ratio=6/12)  # 初始 12×6；palm 会在刷新时按实际形状调整
        view.setMaximumSize(max_w, max_h)
        view.setMinimumSize(min_w, min_h)
        view.setBackground('#0f172a')
        view.hideAxis('left')
        view.hideAxis('bottom')

        img = pg.ImageItem()
        img.setLookupTable(self.colormap.getLookupTable())
        img.setImage(np.zeros((12, 6)), levels=[0, 100])
        view.addItem(img)

        peak_text = pg.TextItem(text="", color=(255, 255, 255), anchor=(0.5, 0.5))
        peak_text.setFont(QtGui.QFont("Arial", 10, QtGui.QFont.Bold))
        view.addItem(peak_text)

        vbox.addWidget(view, stretch=1)

        self.matrix_plots[finger] = view
        self.matrix_images[finger] = img
        self.matrix_peaks[finger] = peak_text
        return container

    def setup_ros(self):
        self.get_logger().info("Setting up ROS...")
        self.subscribe_waveform("/cb_left_hand_matrix_touch_mass")
        self.subscribe_matrix("/cb_left_hand_matrix_touch")

    def subscribe_waveform(self, topic):
        if self.wave_sub:
            self.destroy_subscription(self.wave_sub)
        self.wave_sub = self.create_subscription(String, topic, self.wave_callback, self.qos_profile)
        self.get_logger().info(f"WAVEFORM: {topic}")

    def subscribe_matrix(self, topic):
        if self.matrix_sub:
            self.destroy_subscription(self.matrix_sub)
        self.matrix_sub = self.create_subscription(String, topic, self.matrix_callback, self.qos_profile)
        self.get_logger().info(f"MATRIX: {topic}")

    def switch_wave_topic(self, topic):
        self.subscribe_waveform(topic)
        for f in self.fingers_mass:
            self.wave_data[f] = np.zeros(100)
            self.current_wave_values[f] = 0.0
        # 重置 palm 波形状态并隐藏
        self.wave_data[self.palm_mass_key] = np.zeros(100)
        self.current_wave_values[self.palm_mass_key] = 0.0
        self.palm_wave_present = False
        self.palm_wave_neg_streak = 0
        self._remove_palm_wave_plot()

    def switch_matrix_topic(self, topic):
        self.subscribe_matrix(topic)
        for f in self.fingers_matrix:
            self.matrix_data[f] = np.zeros((12, 6))
            self.matrix_shapes[f] = (12, 6)
            self.data_received[f] = False
        # 重置 palm 热力图状态并隐藏
        self.matrix_data[self.palm_matrix_key] = np.zeros((1, 1))
        self.matrix_shapes[self.palm_matrix_key] = (1, 1)
        self.data_received[self.palm_matrix_key] = False
        self.palm_matrix_present = False
        self.palm_matrix_neg_streak = 0
        if self.palm_heat_container is not None:
            self.palm_heat_container.setVisible(False)

    def wave_callback(self, msg):
        try:
            data = json.loads(msg.data)
            for finger in self.fingers_mass:
                if finger in data:
                    self.current_wave_values[finger] = float(data[finger])

            # palm_mass 可选：不存在则不显示；值为 -1 视为无效帧
            if self.palm_mass_key in data:
                self.palm_wave_present = True
                val = float(data[self.palm_mass_key])
                self.current_wave_values[self.palm_mass_key] = val
                if val == -1:
                    self.palm_wave_neg_streak += 1
                else:
                    self.palm_wave_neg_streak = 0
            else:
                self.palm_wave_present = False
        except Exception as e:
            pass

    def matrix_callback(self, msg):
        """处理矩阵数据（手指为固定形状，palm 尺寸不固定）"""
        try:
            data = json.loads(msg.data)

            for finger in self.fingers_matrix:
                if finger not in data:
                    continue

                arr = np.array(data[finger], dtype=np.float32)

                if arr.ndim == 2:
                    rows, cols = arr.shape
                    self.data_received[finger] = True
                    self.matrix_data[finger] = arr
                    self.matrix_shapes[finger] = (rows, cols)

                    # ROS2 节流日志
                    current_time = self.get_clock().now()
                    if self.last_matrix_log_time[finger] is None or \
                       (current_time - self.last_matrix_log_time[finger]).nanoseconds > 3e9:
                        self.last_matrix_log_time[finger] = current_time
                        self.get_logger().info(f"{finger}: {rows}×{cols}, max={arr.max():.1f}")

            # palm_matrix 可选：不存在则不显示；全为 -1 视为无效帧
            pk = self.palm_matrix_key
            if pk in data:
                arr = np.array(data[pk], dtype=np.float32)
                if arr.ndim == 2:
                    self.palm_matrix_present = True
                    self.matrix_data[pk] = arr
                    self.matrix_shapes[pk] = arr.shape
                    self.data_received[pk] = True
                    if np.all(arr == -1):
                        self.palm_matrix_neg_streak += 1
                    else:
                        self.palm_matrix_neg_streak = 0
            else:
                self.palm_matrix_present = False

        except Exception as e:
            self.get_logger().error(f"Matrix error: {str(e)}")

    def _update_single_heatmap(self, finger):
        """刷新单个热力图（供手指与 palm 复用）"""
        if not self.data_received[finger]:
            return

        data = self.matrix_data[finger]
        rows, cols = self.matrix_shapes[finger]
        img = self.matrix_images[finger]
        view = self.matrix_plots[finger]
        peak_text = self.matrix_peaks[finger]

        max_val = data.max()

        if max_val > 0:
            display_data = data / max_val if max_val > 1 else data
        else:
            display_data = data

        # **关键修复**: 垂直翻转，使第0行(ROS数据第1行)显示在顶部
        display_data_flipped = np.flipud(display_data)
        img.setImage(display_data_flipped, autoLevels=False, levels=[0, 1])

        # palm 尺寸不固定，按当前实际形状锁定纵横比并设置视图范围
        view.setAspectLocked(True, ratio=cols / rows)
        view.setRange(xRange=(-0.5, cols - 0.5), yRange=(-0.5, rows - 0.5))

        # 显示数值：矩阵中的最大压力值，单位g(克)
        if max_val > 0:
            max_idx = np.unravel_index(np.argmax(data), data.shape)
            peak_text.setText(f"{max_val:.0f}")
            # 坐标转换：因为图像翻转了，y坐标也要翻转
            flipped_row = (rows - 1) - max_idx[0]
            peak_text.setPos(max_idx[1], flipped_row)
        else:
            peak_text.setText("0")

    def update_display(self):
        # ===== 波形图 =====
        # 根据最新数据决定 palm 波形是否显示（不存在或连续 palm_hide_frames 帧为 -1 则隐藏）
        show_palm_wave = self.palm_wave_present and \
            self.palm_wave_neg_streak < self.palm_hide_frames
        if show_palm_wave and self.palm_wave_plot is None:
            self._create_palm_wave_plot()
        elif not show_palm_wave and self.palm_wave_plot is not None:
            self._remove_palm_wave_plot()

        wave_fingers = list(self.fingers_mass)
        if self.palm_wave_plot is not None:
            wave_fingers.append(self.palm_mass_key)

        for finger in wave_fingers:
            self.wave_data[finger] = np.roll(self.wave_data[finger], -1)
            self.wave_data[finger][-1] = self.current_wave_values[finger]
            self.wave_curves[finger].setData(self.wave_data[finger])

            val = self.current_wave_values[finger]
            self.wave_curves[finger + '_text'].setText(f"{val:.0f}")
            self.wave_curves[finger + '_text'].setPos(99, val)

        # ===== 热力图 =====
        for finger in self.fingers_matrix:
            self._update_single_heatmap(finger)

        # palm 热力图显隐同步（不存在或连续 palm_hide_frames 帧全为 -1 则隐藏）
        show_palm_matrix = self.palm_matrix_present and \
            self.palm_matrix_neg_streak < self.palm_hide_frames
        if self.palm_heat_container is not None:
            self.palm_heat_container.setVisible(show_palm_matrix)
        if show_palm_matrix:
            self._update_single_heatmap(self.palm_matrix_key)

def signal_handler(sig, frame):
    QtWidgets.QApplication.quit()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    signal.signal(signal.SIGINT, signal_handler)

    gui = PressureDiagram()
    gui.show()

    # ROS2 spin 在 Qt 定时器中处理
    def ros_spin():
        rclpy.spin_once(gui, timeout_sec=0)

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(ros_spin)
    ros_timer.start(1)  # ~1000Hz

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()