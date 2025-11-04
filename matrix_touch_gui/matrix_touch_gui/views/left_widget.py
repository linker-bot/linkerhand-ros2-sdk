import tkinter as tk
from tkinter import ttk, scrolledtext
import time

class LeftControlWidget(ttk.Frame):
    """左侧控制面板部件 - Tkinter版本"""
    
    def __init__(self, parent, hand_type):
        super().__init__(parent)
        self.hand_type = hand_type
        self.subscribe_callback = None
        self.refresh_callback = None
        self.init_ui()
        
    def init_ui(self):
        """初始化左侧控制面板UI"""
        self.configure(width=350)
        
        # 话题选择布局
        topic_frame = ttk.Frame(self)
        topic_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(topic_frame, text="话题:").pack(side=tk.LEFT)
        
        self.topic_var = tk.StringVar()
        self.topic_combo = ttk.Combobox(topic_frame, textvariable=self.topic_var, 
                                       state="readonly", width=25)
        
        # 设置话题列表
        left_topic = "/cb_left_hand_matrix_touch"
        right_topic = "/cb_right_hand_matrix_touch"
        
        if self.hand_type == "right":
            self.topic_combo['values'] = [right_topic, left_topic]
        else:
            self.topic_combo['values'] = [left_topic, right_topic]
            
        if self.topic_combo['values']:
            self.topic_combo.current(0)
        self.topic_combo.pack(side=tk.LEFT, padx=5)
        
        self.subscribe_btn = ttk.Button(topic_frame, text="订阅/取消", 
                                       command=self.on_subscribe_clicked)
        self.subscribe_btn.pack(side=tk.LEFT)
        
        # 刷新频率控制布局
        freq_frame = ttk.Frame(self)
        freq_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(freq_frame, text="刷新频率(ms):").pack(side=tk.LEFT)
        
        self.freq_var = tk.IntVar(value=200)
        self.freq_spinbox = ttk.Spinbox(freq_frame, from_=100, to=1000, 
                                       textvariable=self.freq_var, width=8)
        self.freq_spinbox.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(freq_frame, text="(值越大刷新越慢)").pack(side=tk.LEFT)
        
        # 状态标签
        self.status_label = ttk.Label(self, text="状态: 未订阅 | 红色渐变显示")
        self.status_label.pack(fill=tk.X, pady=5)
        
        # 终端显示
        terminal_frame = ttk.LabelFrame(self, text="终端输出")
        terminal_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.terminal = scrolledtext.ScrolledText(terminal_frame, height=15)
        self.terminal.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.terminal.config(state=tk.DISABLED, bg='black', fg='lime', 
                           font=('Courier', 11))
        
    def on_subscribe_clicked(self):
        """订阅按钮点击事件"""
        if self.subscribe_callback:
            self.subscribe_callback()
            
    def on_refresh_changed(self):
        """刷新频率变化事件"""
        if self.refresh_callback:
            self.refresh_callback(self.freq_var.get())
        
    def get_selected_topic(self):
        """获取选中的话题"""
        return self.topic_var.get()
        
    def update_status(self, status_text, is_subscribed):
        """更新状态标签"""
        self.status_label.config(text=status_text)
        color = "green" if is_subscribed else "red"
        self.status_label.config(foreground=color)
        
    def add_terminal_message(self, text):
        """添加终端消息"""
        timestamp = time.strftime("%H:%M:%S")
        message = f"[{timestamp}] {text}\n"
        
        self.terminal.config(state=tk.NORMAL)
        self.terminal.insert(tk.END, message)
        self.terminal.see(tk.END)
        self.terminal.config(state=tk.DISABLED)
        
    def set_subscribe_callback(self, callback):
        """设置订阅回调函数"""
        self.subscribe_callback = callback
        
    def set_refresh_callback(self, callback):
        """设置刷新回调函数"""
        self.refresh_callback = callback