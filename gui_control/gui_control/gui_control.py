import sys
import time
import json
import threading
from dataclasses import dataclass
from typing import List, Dict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import tkinter.font as tkfont

from .utils.mapping import *
from .config.constants import _HAND_CONFIGS
LOOP_TIME = 1000 # 循环动作间隔时间 毫秒
class ROS2NodeManager:
    """ROS2节点管理器，处理ROS通信"""
    
    def __init__(self, node_name: str = "hand_control_node"):
        self.node = None
        self.publisher = None
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.status_callbacks = []
        
        # 初始化ROS2节点
        self.init_node(node_name)

    def add_status_callback(self, callback):
        """添加状态回调函数"""
        self.status_callbacks.append(callback)

    def emit_status(self, status_type: str, message: str):
        """发射状态信号"""
        for callback in self.status_callbacks:
            callback(status_type, message)

    def init_node(self, node_name: str):
        """初始化ROS2节点"""
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = Node(node_name)
            
            # 声明参数
            self.node.declare_parameter('hand_type', 'left')
            self.node.declare_parameter('hand_joint', 'L10')
            self.node.declare_parameter('topic_hz', 30)
            self.node.declare_parameter('is_arc', False)
            
            # 获取参数
            self.hand_type = self.node.get_parameter('hand_type').value
            self.hand_joint = self.node.get_parameter('hand_joint').value
            self.hz = self.node.get_parameter('topic_hz').value
            self.is_arc = self.node.get_parameter('is_arc').value
            
            if self.is_arc == True:
                # 创建发布者
                self.publisher_arc = self.node.create_publisher(
                    JointState, f'/cb_{self.hand_type}_hand_control_cmd_arc', 10
                )
            # 创建发布者
            self.publisher = self.node.create_publisher(
                JointState, f'/cb_{self.hand_type}_hand_control_cmd', 10
            )
            # 新增 speed / torque 发布者
            self.speed_pub = self.node.create_publisher(
                String, f'/cb_hand_setting_cmd', 10)
            self.torque_pub = self.node.create_publisher(
                String, f'/cb_hand_setting_cmd', 10)
            self.emit_status("info", f"ROS2节点初始化成功: {self.hand_type} {self.hand_joint}")
            
            # 启动ROS2自旋线程
            self.spin_thread = threading.Thread(target=self.spin_node, daemon=True)
            self.spin_thread.start()
        except Exception as e:
            self.emit_status("error", f"ROS2初始化失败: {str(e)}")
            raise

    def spin_node(self):
        """运行ROS2节点自旋循环"""
        while rclpy.ok() and self.node:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def publish_joint_state(self, positions: List[int]):
        """发布关节状态消息"""
        if not self.publisher or not self.node:
            self.emit_status("error", "ROS2发布者未初始化")
            return
            
        try:
            self.joint_state.header.stamp = self.node.get_clock().now().to_msg()
            self.joint_state.position = [float(pos) for pos in positions]
            hand_config = _HAND_CONFIGS[self.hand_joint]
            if len(hand_config.joint_names) == len(positions):
                if hand_config.joint_names_en != None:
                    self.joint_state.name = hand_config.joint_names_en
                else:
                    self.joint_state.name = hand_config.joint_names
                
            self.publisher.publish(self.joint_state)
            if self.is_arc == True:
                if self.hand_joint == "O6":
                    if self.hand_type == "left":
                        pose = range_to_arc_left(positions,self.hand_joint)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right(positions,self.hand_joint)
                elif self.hand_joint == "L7" or self.hand_joint == "L21" or self.hand_joint == "L25":
                    if self.hand_type == "left":
                        pose = range_to_arc_left(positions,self.hand_joint)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right(positions,self.hand_joint)
                elif self.hand_joint == "L10":
                    if self.hand_type == "left":
                        pose = range_to_arc_left_10(positions)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right_10(positions)
                elif self.hand_joint == "L20":
                    if self.hand_type == "left":
                        pose = range_to_arc_left_l20(positions)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right_l20(positions)
                else:
                    print(f"当前{self.hand_joint} {self.hand_type}不支持弧度转换", flush=True)
                self.joint_state.position = [float(pos) for pos in pose]
                self.publisher_arc.publish(self.joint_state)
            self.emit_status("info", "关节状态已发布")
        except Exception as e:
            self.emit_status("error", f"发布失败: {str(e)}")

    def publish_speed(self, val: int):
        joint_len = 0
        if (self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6"):
            joint_len = 6
        elif self.hand_joint == "L7":
            joint_len = 7
        elif self.hand_joint == "L10":
            joint_len = 10
        else:
            joint_len = 5
        msg = String()
        v = [val] * joint_len
        data = {
            "setting_cmd": "set_speed",
            "params": {"hand_type":self.hand_type,"speed": v},
        }
        msg.data = json.dumps(data)
        print(f"速度值：{v}", flush=True)
        self.speed_pub.publish(msg)

    def publish_torque(self, val: int):
        joint_len = 0
        if (self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6"):
            joint_len = 6
        elif self.hand_joint == "L7":
            joint_len = 7
        elif self.hand_joint == "L10":
            joint_len = 10
        else:
            joint_len = 5
        msg = String()
        v = [val] * joint_len
        data = {
            "setting_cmd": "set_max_torque_limits",
            "params": {"hand_type":self.hand_type,"torque": v},
        }
        
        msg.data = json.dumps(data)
        print(f"扭矩值：{v}", flush=True)
        self.torque_pub.publish(msg)

    def shutdown(self):
        """关闭ROS2节点"""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

class HandControlGUI:
    """灵巧手控制界面"""
    
    def __init__(self, ros_manager: ROS2NodeManager):
        self.ros_manager = ros_manager
        self.ros_manager.add_status_callback(self.update_status)
        
        # 获取手部配置
        self.hand_joint = self.ros_manager.hand_joint
        self.hand_type = self.ros_manager.hand_type
        self.hand_config = _HAND_CONFIGS[self.hand_joint]
        
        # 循环控制变量
        self.cycle_timer = None
        self.current_action_index = -1
        self.preset_buttons = []
        
        # 初始化UI
        self.init_ui()
        
        # 设置定时器发布关节状态
        self.publish_timer_id = None
        self.start_publish_timer()

    def init_ui(self):
        """初始化用户界面"""
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title(f'灵巧手控制界面 - {self.hand_type} {self.hand_joint}')
        self.root.geometry('1300x700')
        
        # 设置样式
        self.style = ttk.Style()
        self.style.configure('TFrame', background='#f0f0f0')
        self.style.configure('TLabel', background='#f0f0f0', font=('Microsoft YaHei', 10))
        self.style.configure('Title.TLabel', font=('Microsoft YaHei', 14, 'bold'), foreground='#165DFF')
        self.style.configure('Group.TLabelframe', borderwidth=2, relief='groove')
        self.style.configure('Group.TLabelframe.Label', font=('Microsoft YaHei', 10, 'bold'), foreground='#165DFF')
        # 配置信息框样式 - 灰色背景
        self.style.configure('Info.TFrame', background='#e0e0e0')
        # 配置高亮样式 - 滑动时的背景色
        self.style.configure('Highlight.TFrame', background='#e6f7ff')  # 浅蓝色背景
        
        # 初始化当前点击按钮索引
        self.current_clicked_button = None
        
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建水平分割的框架
        self.paned_window = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True)
        
        # 创建左侧关节控制面板
        self.joint_control_panel = self.create_joint_control_panel()
        self.paned_window.add(self.joint_control_panel, weight=5)
        
        # 创建中间预设动作面板
        self.preset_actions_panel = self.create_preset_actions_panel()
        self.paned_window.add(self.preset_actions_panel, weight=3)
        
        # 创建右侧状态监控面板
        self.status_monitor_panel = self.create_status_monitor_panel()
        self.paned_window.add(self.status_monitor_panel, weight=4)
        
        # 创建底部数值显示面板
        self.value_display_panel = self.create_value_display_panel()
        self.value_display_panel.pack(fill=tk.X, pady=(10, 0))
        
        # 绑定窗口大小变化事件
        self.joint_control_panel.bind('<Configure>', self.on_joint_panel_resize)
        
        # 初始更新数值显示
        self.update_value_display()

    def create_joint_control_panel(self):
        """创建关节控制面板"""
        frame = ttk.Frame(self.root)
        frame.config(width=600)  # 设置最小宽度为400像素
        frame.pack_propagate(False)  # 阻止子控件改变框架大小
        # 创建标题
        title_label = ttk.Label(frame, text=f"关节控制 - {self.hand_joint}", style='Title.TLabel')
        title_label.pack(pady=(0, 10))
        
        # 创建滚动框架
        self.canvas = tk.Canvas(frame, bg='#f0f0f0', highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)
        
        self.scrollable_frame.bind(
            "<Configure>",
            self.on_scrollable_frame_configure  # 修改为调用方法
        )
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        # 创建滑动条
        self.create_joint_sliders(self.scrollable_frame)
        
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        # 初始时不显示滚动条
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.scrollbar.pack_forget()  # 隐藏滚动条
        
        return frame
    def on_scrollable_frame_configure(self, event):
        """滚动区域配置变化事件 - 动态显示/隐藏滚动条"""
        # 更新滚动区域
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        
        # 检查是否需要显示滚动条
        scrollable_height = self.scrollable_frame.winfo_reqheight()  # 内容所需高度
        canvas_height = self.canvas.winfo_height()  # 画布实际高度
        
        # 如果内容高度大于画布高度，显示滚动条；否则隐藏
        if scrollable_height > canvas_height and canvas_height > 0:
            self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        else:
            self.scrollbar.pack_forget()

    def create_joint_sliders(self, parent):
        """创建关节滑动条"""
        self.sliders = []
        self.slider_labels = []
        self.slider_frames = []
        
        # 存储父框架的引用，用于后续调整大小
        self.sliders_parent = parent
        
        for i, (name, value) in enumerate(zip(
            self.hand_config.joint_names, self.hand_config.init_pos
        )):
            # 创建框架 - 添加左右边距
            slider_frame = ttk.Frame(parent)
            slider_frame.pack(fill=tk.X, pady=5, padx=(10,0))  # 左右各10像素边距
            self.slider_frames.append(slider_frame)
            
            # 创建标签
            label = ttk.Label(slider_frame, text=f"{name}: {value}", width=15)
            label.pack(side=tk.LEFT, padx=(0, 10))
            
            # 创建滑动条 - 初始长度设为300，但会随窗口调整
            slider = ttk.Scale(slider_frame, from_=0, to=255, value=value, 
                              orient=tk.HORIZONTAL, length=300,  # 初始长度
                              command=lambda val, idx=i: self.on_slider_value_changed(idx, val))
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            self.sliders.append(slider)
            self.slider_labels.append(label)

    def on_joint_panel_resize(self, event):
        """关节控制面板大小变化事件"""
        # 获取面板当前宽度
        panel_width = event.width
        
        # 为所有滑动条设置新的长度
        for slider in self.sliders:
            # 计算新的滑动条长度（面板宽度减去标签和其他元素的估计宽度）
            # 标签宽度约120像素 + 左右边距各10像素 + 标签与滑动条间距10像素
            new_length = max(panel_width - 150, 100)  # 最小长度100像素
            # 重新配置滑动条
            slider.configure(length=new_length-30)
        # 面板大小变化后重新检查是否需要滚动条
        self.root.after(100, self.check_scrollbar_visibility)  # 延迟检查，确保高度已更新

    def check_scrollbar_visibility(self):
        """检查滚动条可见性"""
        # 只有在画布已经有实际高度时才检查
        if self.canvas.winfo_height() > 0:
            scrollable_height = self.scrollable_frame.winfo_reqheight()
            canvas_height = self.canvas.winfo_height()
            
            if scrollable_height > canvas_height:
                self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            else:
                self.scrollbar.pack_forget()

    def create_preset_actions_panel(self):
        """创建预设动作面板"""
        frame = ttk.Frame(self.root)
        
        # 系统预设动作
        sys_preset_group = ttk.LabelFrame(frame, text="系统预设", style='Group.TLabelframe')
        sys_preset_group.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # 创建系统预设动作按钮 - 3列布局
        self.create_system_preset_buttons(sys_preset_group)
        
        # 动作按钮框架
        actions_frame = ttk.Frame(frame)
        actions_frame.pack(fill=tk.X, pady=10)
        
        # 循环运行按钮
        self.cycle_button = ttk.Button(actions_frame, text="循环预设动作", 
                                      command=self.on_cycle_clicked, width=12)
        self.cycle_button.pack(side=tk.LEFT, padx=5)
        
        # 回到初始位置按钮
        self.home_button = ttk.Button(actions_frame, text="回到初始位置", 
                                     command=self.on_home_clicked, width=12)
        self.home_button.pack(side=tk.LEFT, padx=5)
        
        # 停止所有动作按钮
        self.stop_button = ttk.Button(actions_frame, text="停止所有动作", 
                                     command=self.on_stop_clicked, width=12)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        return frame

    def create_system_preset_buttons(self, parent):
        """创建系统预设动作按钮 - 3列布局"""
        self.preset_buttons = []
        if self.hand_config.preset_actions:
            buttons_frame = ttk.Frame(parent)
            buttons_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            buttons = []
            for idx, (name, positions) in enumerate(self.hand_config.preset_actions.items()):
                # 使用 tk.Button 而不是 ttk.Button，以便更好地控制背景色
                button = tk.Button(buttons_frame, text=name, bg='#E6F7FF', fg='#1890FF', width=15,
                                relief='raised', bd=1, font=('Microsoft YaHei', 10),
                                command=lambda pos=positions, btn_idx=idx: self.on_preset_action_clicked(pos, btn_idx))
                buttons.append(button)
                self.preset_buttons.append(button)
            
            # 3列布局
            cols = 3
            for i, button in enumerate(buttons):
                row, col = divmod(i, cols)
                button.grid(row=row, column=col, sticky='ew', padx=5, pady=5)
            
            # 配置列权重
            for i in range(cols):
                buttons_frame.columnconfigure(i, weight=1)

    def create_status_monitor_panel(self):
        """创建状态监控面板"""
        frame = ttk.Frame(self.root)
        
        # 标题
        title_label = ttk.Label(frame, text="状态监控", style='Title.TLabel')
        title_label.pack(pady=(0, 10))
        
        # 快速设置框架
        quick_set_group = ttk.LabelFrame(frame, text="快速设置", style='Group.TLabelframe')
        quick_set_group.pack(fill=tk.X, pady=(0, 10))
        
        # 速度设置
        speed_frame = ttk.Frame(quick_set_group)
        speed_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(speed_frame, text="速度:").pack(side=tk.LEFT)
        self.speed_var = tk.IntVar(value=255)
        self.speed_slider = ttk.Scale(speed_frame, from_=0, to=255, 
                                     variable=self.speed_var, orient=tk.HORIZONTAL)
        self.speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.speed_val_label = ttk.Label(speed_frame, text="255", width=4)
        self.speed_val_label.pack(side=tk.LEFT)
        self.speed_btn = ttk.Button(speed_frame, text="设置速度",
                                   command=self.on_speed_set)
        self.speed_btn.pack(side=tk.LEFT, padx=5)
        
        # 扭矩设置
        torque_frame = ttk.Frame(quick_set_group)
        torque_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(torque_frame, text="扭矩:").pack(side=tk.LEFT)
        self.torque_var = tk.IntVar(value=255)
        self.torque_slider = ttk.Scale(torque_frame, from_=0, to=255, 
                                      variable=self.torque_var, orient=tk.HORIZONTAL)
        self.torque_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.torque_val_label = ttk.Label(torque_frame, text="255", width=4)
        self.torque_val_label.pack(side=tk.LEFT)
        self.torque_btn = ttk.Button(torque_frame, text="设置扭矩",
                                    command=self.on_torque_set)
        self.torque_btn.pack(side=tk.LEFT, padx=5)
        
        # 绑定滑块值变化事件
        self.speed_var.trace('w', self.on_speed_changed)
        self.torque_var.trace('w', self.on_torque_changed)
        
        # 创建标签页
        notebook = ttk.Notebook(frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # 系统信息标签页
        sys_info_frame = ttk.Frame(notebook)
        notebook.add(sys_info_frame, text="系统信息")
        
        # 连接状态 - 灰色背景，左对齐
        conn_group = ttk.LabelFrame(sys_info_frame, text="连接状态", style='Group.TLabelframe')
        conn_group.pack(fill=tk.X, pady=5)
        
        # 创建灰色背景的框架
        conn_content_frame = ttk.Frame(conn_group, style='Info.TFrame')
        conn_content_frame.pack(fill=tk.X, padx=10, pady=10)
        
        if self.ros_manager.publisher.get_subscription_count() > 0:
            self.connection_status = ttk.Label(conn_content_frame, text="ROS2节点已连接", 
                                              foreground="green", background='#e0e0e0',
                                              anchor='w')  # 左对齐
        else:
            self.connection_status = ttk.Label(conn_content_frame, text="ROS2节点未连接", 
                                              foreground="red", background='#e0e0e0',
                                              anchor='w')  # 左对齐
        self.connection_status.pack(fill=tk.X)
        
        # 手部信息 - 灰色背景，左对齐
        hand_info_group = ttk.LabelFrame(sys_info_frame, text="手部信息", style='Group.TLabelframe')
        hand_info_group.pack(fill=tk.X, pady=5)
        
        # 创建灰色背景的框架
        hand_info_content_frame = ttk.Frame(hand_info_group, style='Info.TFrame')
        hand_info_content_frame.pack(fill=tk.X, padx=10, pady=10)
        
        info_text = f"""手部类型: {self.hand_type}
关节型号: {self.hand_joint}
关节数量: {len(self.hand_config.joint_names)}
发布频率: {self.ros_manager.hz} Hz"""
        self.hand_info_label = ttk.Label(hand_info_content_frame, text=info_text,
                                        background='#e0e0e0', anchor='w', justify='left')  # 左对齐
        self.hand_info_label.pack(fill=tk.X)
        
        # 状态日志标签页
        log_frame = ttk.Frame(notebook)
        notebook.add(log_frame, text="状态日志")
        
        # 日志文本框
        self.status_log = scrolledtext.ScrolledText(log_frame, height=15, width=50)
        self.status_log.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.status_log.insert(tk.END, "等待系统启动...\n")
        self.status_log.config(state=tk.DISABLED)
        
        # 清除日志按钮
        clear_log_btn = ttk.Button(log_frame, text="清除日志", 
                                  command=self.clear_status_log)
        clear_log_btn.pack(pady=5)
        
        return frame

    def create_value_display_panel(self):
        """创建滑动条数值显示面板"""
        frame = ttk.LabelFrame(self.root, text="关节数值列表", style='Group.TLabelframe')
        
        # 创建按钮框架（放在显示框上方）
        button_frame = ttk.Frame(frame)
        button_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        
        # 复制按钮 - 左对齐
        copy_button = ttk.Button(button_frame, text="复制到剪切板", 
                            command=self.copy_values_to_clipboard, width=10)
        copy_button.pack(side=tk.LEFT)
        
        # 数值显示框
        self.value_display = scrolledtext.ScrolledText(frame, height=4, width=100)
        self.value_display.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        self.value_display.insert(tk.END, "[]")
        self.value_display.config(state=tk.DISABLED)
        
        return frame

    def copy_values_to_clipboard(self):
        """复制关节数值到系统剪切板"""
        try:
            # 获取当前按钮引用
            button = self.root.focus_get()
            original_text = "复制到剪切板"
            
            # 获取文本框内容
            content = self.value_display.get(1.0, tk.END).strip()
            
            # 清除文本框的选中状态
            self.value_display.tag_remove(tk.SEL, "1.0", tk.END)
            
            # 复制到剪切板
            self.root.clipboard_clear()
            self.root.clipboard_append(content)
            
            # 改变按钮文本提示复制成功
            if isinstance(button, ttk.Button):
                button.config(text="已复制!")
                # 1.5秒后恢复原文本
                self.root.after(1500, lambda: button.config(text=original_text))
            
            self.update_status("info", f"关节数值已复制到剪切板")
            
        except Exception as e:
            self.update_status("error", f"复制失败: {str(e)}")

    def on_slider_value_changed(self, index: int, value: str):
        """滑动条值改变事件处理"""
        value_int = int(float(value))
        if 0 <= index < len(self.slider_labels):
            joint_name = self.hand_config.joint_names[index]
            self.slider_labels[index].config(text=f"{joint_name}: {value_int}")
            
        # 更新数值显示
        self.update_value_display()

    def update_value_display(self):
        """更新数值显示面板内容"""
        values = [int(float(slider.get())) for slider in self.sliders]
        
        self.value_display.config(state=tk.NORMAL)
        self.value_display.delete(1.0, tk.END)
        self.value_display.insert(tk.END, f"{values}")
        self.value_display.config(state=tk.DISABLED)

    def on_preset_action_clicked(self, positions: List[int], button_index: int = None):
        """预设动作按钮点击事件处理"""
        if len(positions) != len(self.sliders):
            messagebox.showwarning(
                "动作不匹配", 
                f"预设动作关节数量({len(positions)})与当前关节数量({len(self.sliders)})不匹配"
            )
            return
        
        # 重置所有按钮颜色
        self.reset_preset_buttons_color()
        
        # 高亮当前点击的按钮
        if button_index is not None and 0 <= button_index < len(self.preset_buttons):
            self.preset_buttons[button_index].config(bg='#1890FF', fg='white')
            self.current_clicked_button = button_index
        
        # 更新滑动条
        for i, (slider, pos) in enumerate(zip(self.sliders, positions)):
            slider.set(pos)
            self.on_slider_value_changed(i, str(pos))
            
        # 发布关节状态
        self.publish_joint_state()

    def on_home_clicked(self):
        """回到初始位置按钮点击事件处理"""
        for slider, pos in zip(self.sliders, self.hand_config.init_pos):
            slider.set(pos)
            
        self.publish_joint_state()
        self.update_status("info", "回到初始位置")
        
        # 更新数值显示
        self.update_value_display()

    def on_stop_clicked(self):
        """停止所有动作按钮点击事件处理"""
        # 停止循环定时器
        if self.cycle_timer:
            self.root.after_cancel(self.cycle_timer)
            self.cycle_timer = None
            self.cycle_button.config(text="循环运行预设动作")
            self.reset_preset_buttons_color()
            
        self.update_status("warning", "已停止所有动作")

    def on_cycle_clicked(self):
        """循环运行预设动作按钮点击事件处理"""
        if not self.hand_config.preset_actions:
            messagebox.showwarning("无预设动作", "当前手部型号没有预设动作可循环运行")
            return
            
        if self.cycle_timer:
            # 停止循环
            self.root.after_cancel(self.cycle_timer)
            self.cycle_timer = None
            self.cycle_button.config(text="循环运行预设动作")
            self.reset_preset_buttons_color()
            # 如果有之前点击的按钮，恢复其点击状态
            if hasattr(self, 'current_clicked_button') and self.current_clicked_button is not None:
                if 0 <= self.current_clicked_button < len(self.preset_buttons):
                    self.preset_buttons[self.current_clicked_button].config(bg='#1890FF', fg='white')
            self.update_status("info", "已停止循环运行预设动作")
        else:
            # 开始循环
            self.current_action_index = -1
            self.cycle_button.config(text="停止循环运行")
            self.update_status("info", "开始循环运行预设动作")
            self.run_next_action()

    def run_next_action(self):
        """运行下一个预设动作"""
        if not self.hand_config.preset_actions:
            return
            
        # 重置所有按钮颜色
        self.reset_preset_buttons_color()
        
        # 计算下一个动作索引
        self.current_action_index = (self.current_action_index + 1) % len(self.hand_config.preset_actions)
        
        # 获取下一个动作
        action_names = list(self.hand_config.preset_actions.keys())
        action_name = action_names[self.current_action_index]
        action_positions = self.hand_config.preset_actions[action_name]
        
        # 执行动作
        self.on_preset_action_clicked(action_positions, self.current_action_index)
        
        # 高亮当前动作按钮（循环模式使用不同的颜色）
        if 0 <= self.current_action_index < len(self.preset_buttons):
            button = self.preset_buttons[self.current_action_index]
            button.config(bg='#52C41A', fg='white')  # 绿色表示循环中的按钮
        
        self.update_status("info", f"运行预设动作: {action_name}")
        
        # 设置下一个动作定时器
        self.cycle_timer = self.root.after(LOOP_TIME, self.run_next_action)

    def reset_preset_buttons_color(self):
        """重置所有预设按钮颜色"""
        for button in self.preset_buttons:
            button.config(bg='#E6F7FF', fg='#1890FF')  # 恢复默认颜色

    def on_speed_changed(self, *args):
        """速度滑块值改变事件"""
        self.speed_val_label.config(text=str(self.speed_var.get()))

    def on_torque_changed(self, *args):
        """扭矩滑块值改变事件"""
        self.torque_val_label.config(text=str(self.torque_var.get()))

    def on_speed_set(self):
        """设置速度"""
        speed_val = self.speed_var.get()
        self.ros_manager.publish_speed(speed_val)
        self.update_status("info", f"速度已设为 {speed_val}")

    def on_torque_set(self):
        """设置扭矩"""
        torque_val = self.torque_var.get()
        self.ros_manager.publish_torque(torque_val)
        self.update_status("info", f"扭矩已设为 {torque_val}")

    def publish_joint_state(self):
        """发布当前关节状态"""
        positions = [int(float(slider.get())) for slider in self.sliders]
        self.ros_manager.publish_joint_state(positions)

    def start_publish_timer(self):
        """开始发布定时器"""
        self.publish_joint_state()
        interval = int(1000 / self.ros_manager.hz)
        self.publish_timer_id = self.root.after(interval, self.start_publish_timer)

    def update_status(self, status_type: str, message: str):
        """更新状态显示"""
        # 更新连接状态
        if status_type == "info" and "ROS2节点初始化成功" in message:
            self.connection_status.config(text="ROS2节点已连接", foreground="green")
            
        # 更新日志
        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        
        self.status_log.config(state=tk.NORMAL)
        self.status_log.insert(tk.END, log_entry)
        self.status_log.see(tk.END)
        
        # 限制日志长度
        log_content = self.status_log.get(1.0, tk.END)
        if len(log_content) > 10000:
            self.status_log.delete(1.0, f"{len(log_content)-10000}.0")
            
        self.status_log.config(state=tk.DISABLED)
        
        # 设置日志颜色
        if status_type == "error":
            # 可以在Tkinter中为不同消息类型添加颜色标记
            pass

    def clear_status_log(self):
        """清除状态日志"""
        self.status_log.config(state=tk.NORMAL)
        self.status_log.delete(1.0, tk.END)
        self.status_log.insert(tk.END, "日志已清除\n")
        self.status_log.config(state=tk.DISABLED)

    def run(self):
        """运行GUI"""
        try:
            self.root.mainloop()
        finally:
            # 清理定时器
            if self.cycle_timer:
                self.root.after_cancel(self.cycle_timer)
            if self.publish_timer_id:
                self.root.after_cancel(self.publish_timer_id)
            # 关闭ROS2节点
            self.ros_manager.shutdown()

def main(args=None):
    """主函数"""
    try:
        # 创建ROS2节点管理器
        ros_manager = ROS2NodeManager()
        
        # 创建GUI
        gui = HandControlGUI(ros_manager)
        
        # 运行应用
        gui.run()
            
    except Exception as e:
        print(f"应用程序启动失败: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main()
