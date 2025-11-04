import tkinter as tk
from tkinter import ttk

class DotMatrixWidget(tk.Canvas):
    """点阵显示部件 - 白色到深红色渐变版"""
    
    def __init__(self, parent, rows=12, cols=6):
        self.rows = rows
        self.cols = cols
        self.dot_size = 8
        self.spacing = 4
        
        # 计算画布大小
        width = cols * (self.dot_size + self.spacing) + self.spacing
        height = rows * (self.dot_size + self.spacing) + self.spacing
        
        super().__init__(parent, width=width, height=height, 
                        bg='white', highlightthickness=1, 
                        highlightbackground='#cccccc')
        
        self.data = None
        # 初始化时显示默认点阵
        self.draw_default_matrix()
        
    def draw_default_matrix(self):
        """绘制默认点阵（所有点为灰色）"""
        self.delete("all")
        for row in range(self.rows):
            for col in range(self.cols):
                x = self.spacing + col * (self.dot_size + self.spacing)
                y = self.spacing + row * (self.dot_size + self.spacing)
                # 默认显示灰色点
                self.create_oval(x, y, x + self.dot_size, y + self.dot_size, 
                                fill='#c8c8c8', outline='#666666', width=1)
        
    def set_data(self, data):
        """设置点阵数据"""
        self.data = data
        self.draw_matrix()
        
    def draw_matrix(self):
        """绘制点阵 - 白色到深红色渐变"""
        self.delete("all")
        
        for row in range(self.rows):
            for col in range(self.cols):
                x = self.spacing + col * (self.dot_size + self.spacing)
                y = self.spacing + row * (self.dot_size + self.spacing)
                
                if self.data and isinstance(self.data, list) and len(self.data) > row * self.cols + col:
                    try:
                        value = self.data[row * self.cols + col]
                        if value > 0:
                            # 白色到深红色渐变逻辑
                            # 数值范围假设为0-255，数值越大红色越深
                            intensity = min(255, max(0, int(value)))
                            
                            if intensity < 128:
                                # 白色到浅红色渐变
                                # 白色 (255,255,255) -> 浅红色 (255,200,200)
                                red = 255
                                green = 255 - (intensity * 55 // 128)
                                blue = 255 - (intensity * 55 // 128)
                                color = f'#{red:02x}{green:02x}{blue:02x}'
                            else:
                                # 浅红色到深红色渐变
                                # 浅红色 (255,200,200) -> 深红色 (255,0,0)
                                red = 255
                                green = 200 - ((intensity - 128) * 200 // 127)
                                blue = 200 - ((intensity - 128) * 200 // 127)
                                color = f'#{red:02x}{green:02x}{blue:02x}'
                        else:
                            color = '#c8c8c8'  # 默认灰色
                    except:
                        color = '#c8c8c8'  # 默认灰色
                else:
                    color = '#c8c8c8'  # 默认灰色
                
                # 绘制圆点
                self.create_oval(x, y, x + self.dot_size, y + self.dot_size, 
                                fill=color, outline='#666666', width=1)

class RightMatrixWidget(ttk.Frame):
    """右侧矩阵显示部件 - Tkinter版本"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.finger_matrices = {}
        self.init_ui()
        
    def init_ui(self):
        """初始化右侧矩阵UI"""
        main_layout = ttk.Frame(self)
        main_layout.pack(fill=tk.BOTH, expand=True)
        
        # 五个手指的点阵
        finger_names = ["thumb_matrix", "index_matrix", "middle_matrix", "ring_matrix", "little_matrix"]
        display_names = ["拇指", "食指", "中指", "无名指", "小指"]
        
        for finger_name, display_name in zip(finger_names, display_names):
            # 创建手指布局
            finger_frame = ttk.Frame(main_layout)
            finger_frame.pack(fill=tk.X, pady=2)
            
            # 手指标签
            ttk.Label(finger_frame, text=display_name).pack()
            
            # 创建点阵部件 - 初始化时显示默认灰色点阵
            matrix = DotMatrixWidget(finger_frame)
            matrix.pack(pady=2)
            
            self.finger_matrices[finger_name] = matrix
            
    def update_matrix_data(self, finger_name, data):
        """更新指定手指的点阵数据"""
        if finger_name in self.finger_matrices:
            # 展平数据（如果数据是二维列表）
            if data and isinstance(data[0], list):
                flattened = [item for sublist in data for item in sublist]
            else:
                flattened = data
                
            self.finger_matrices[finger_name].set_data(flattened)