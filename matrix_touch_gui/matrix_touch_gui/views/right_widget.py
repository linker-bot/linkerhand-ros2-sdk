from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QLabel)

class DotMatrixWidget(QWidget):
    """点阵显示部件 - 红色渐变版"""
    def __init__(self, rows=12, cols=6, parent=None):
        super().__init__(parent)
        self.rows = rows
        self.cols = cols
        self.dot_size = 8
        self.spacing = 4
        self.data = None
        self.setMinimumSize(
            cols * (self.dot_size + self.spacing) + self.spacing,
            rows * (self.dot_size + self.spacing) + self.spacing
        )

    def set_data(self, data):
        """设置点阵数据"""
        self.data = data
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(QColor(0, 0, 0), 1)
        painter.setPen(pen)
        default_brush = QBrush(QColor(200, 200, 200))  # 灰色默认点

        for row in range(self.rows):
            for col in range(self.cols):
                x = self.spacing + col * (self.dot_size + self.spacing)
                y = self.spacing + row * (self.dot_size + self.spacing)
                
                if self.data and isinstance(self.data, list) and len(self.data) > row * self.cols + col:
                    try:
                        value = self.data[row * self.cols + col]
                        if value > 0:
                            # 红色渐变逻辑：数值越大红色越深
                            alpha = min(255, max(0, int(value)))  # 确保alpha值在0-255范围内
                            red_color = QColor(255, 0, 0)  # 红色
                            red_color.setAlpha(alpha)
                            painter.setBrush(QBrush(red_color))
                        else:
                            painter.setBrush(default_brush)
                    except:
                        painter.setBrush(default_brush)
                else:
                    painter.setBrush(default_brush)
                    
                painter.drawEllipse(x, y, self.dot_size, self.dot_size)

class RightMatrixWidget(QWidget):
    """右侧矩阵显示部件"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.finger_matrices = {}
        self.init_ui()
        
    def init_ui(self):
        """初始化右侧矩阵UI"""
        main_layout = QVBoxLayout(self)
        
        # 五个手指的点阵
        finger_names = ["thumb_matrix", "index_matrix", "middle_matrix", "ring_matrix", "little_matrix"]
        display_names = ["拇指", "食指", "中指", "无名指", "小指"]
        
        for finger_name, display_name in zip(finger_names, display_names):
            finger_layout = QVBoxLayout()
            finger_layout.addWidget(QLabel(display_name))
            matrix = DotMatrixWidget()
            finger_layout.addWidget(matrix)
            main_layout.addLayout(finger_layout)
            self.finger_matrices[finger_name] = matrix
            
        # 添加拉伸因子，使布局更美观
        main_layout.addStretch()
        
    def update_matrix_data(self, finger_name, data):
        """更新指定手指的点阵数据"""
        if finger_name in self.finger_matrices:
            flattened = [item for sublist in data for item in sublist]
            self.finger_matrices[finger_name].set_data(flattened)