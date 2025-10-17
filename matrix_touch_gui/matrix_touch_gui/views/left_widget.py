
import sys
from PyQt5.QtWidgets import ( QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit, QPushButton, QComboBox, QSpinBox)
from PyQt5.QtCore import  pyqtSignal

class LeftControlWidget(QWidget):
    """左侧控制面板部件"""
    subscribe_clicked = pyqtSignal()
    refresh_interval_changed = pyqtSignal(int)
    
    def __init__(self, hand_type, parent=None):
        super().__init__(parent)
        self.hand_type = hand_type
        self.init_ui()
        
    def init_ui(self):
        """初始化左侧控制面板UI"""
        main_layout = QVBoxLayout(self)
        self.setMinimumWidth(350)
        
        # 话题选择布局
        topic_layout = QHBoxLayout()
        self.topic_combo = QComboBox()
        left_topic = "/cb_right_hand_matrix_touch"
        right_topic = "/cb_left_hand_matrix_touch"
        
        if self.hand_type == "right":
            self.topic_combo.addItems([left_topic, right_topic])
        else:
            self.topic_combo.addItems([right_topic, left_topic])
            
        self.subscribe_btn = QPushButton("订阅/取消")
        self.subscribe_btn.clicked.connect(self.subscribe_clicked.emit)
        
        topic_layout.addWidget(QLabel("话题:"))
        topic_layout.addWidget(self.topic_combo)
        topic_layout.addWidget(self.subscribe_btn)
        main_layout.addLayout(topic_layout)
        
        # 刷新频率控制布局
        freq_layout = QHBoxLayout()
        self.freq_spinbox = QSpinBox()
        self.freq_spinbox.setRange(100, 1000)
        self.freq_spinbox.setValue(200)
        self.freq_spinbox.valueChanged.connect(self.on_refresh_changed)
        
        freq_layout.addWidget(QLabel("刷新频率(ms):"))
        freq_layout.addWidget(self.freq_spinbox)
        freq_layout.addWidget(QLabel("(值越大刷新越慢)"))
        main_layout.addLayout(freq_layout)
        
        # 状态标签
        self.status_label = QLabel("状态: 未订阅 | 红色渐变显示")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        main_layout.addWidget(self.status_label)
        
        # 终端显示
        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setStyleSheet("background-color: black; color: lime; font-family: monospace;")
        main_layout.addWidget(self.terminal)
        
    def on_refresh_changed(self, value):
        """刷新频率变化时发射信号"""
        self.refresh_interval_changed.emit(value)
        
    def get_selected_topic(self):
        """获取选中的话题"""
        return self.topic_combo.currentText()
        
    def update_status(self, status_text, is_subscribed):
        """更新状态标签"""
        self.status_label.setText(status_text)
        self.status_label.setStyleSheet(f"color: {'green' if is_subscribed else 'red'}; font-weight: bold;")
        
    def add_terminal_message(self, text):
        """添加终端消息"""
        import time
        timestamp = time.strftime("%H:%M:%S")
        self.terminal.append(f"[{timestamp}] {text}")
        self.terminal.moveCursor(self.terminal.textCursor().End)