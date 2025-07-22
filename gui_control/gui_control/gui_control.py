import sys
import rclpy,time,threading
from rclpy.node import Node
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import JointState

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider,
    QLabel, QPushButton, QLineEdit, QGridLayout, QScrollArea
)

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L10')
        self.declare_parameter('topic_hz', 30)
        self.declare_parameter('is_touch', False)
        
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.hz = self.get_parameter('topic_hz').value
        self.is_touch = self.get_parameter('is_touch').value
        self.last_msg = []
        self.publisher = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_control_cmd', 10)
    def get_hand(self):
        return self.hand_type,self.hand_joint,self.hz,self.is_touch
    def publish_control_cmd(self, msg):
        self.publisher.publish(msg)

class GuiApp(QWidget):
    handle_button_click = pyqtSignal(str)
    add_button_handle = pyqtSignal(str)

    def __init__(self,hand_type="left",hand_joint="L10",hz=30):
        super().__init__()
        self.hand_type = hand_type
        self.hand_joint = hand_joint
        self.interval = 1.0 / hz  # 每次间隔 0.033 秒
        self.last_msg = JointState()
        #self.yaml = LoadWriteYaml()
        self.setWindowTitle(f'ROS2 Control Linker Hand {hand_type} {hand_joint}')
        self.setFixedSize(800, 900)
        self.node = None
        self.buttons = []
        self.control_sliders = []
        self.slider_labels = {}
        self.row = 0
        self.column = 0
        self.BUTTONS_PER_ROW = 3  # 每行最多 3 个按钮

        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # 左侧滑动条
        self.left_layout = QVBoxLayout()
        self.slider_list = ['Slider 1', 'Slider 2', 'Slider 3']
        self.init_hand()
        self.create_sliders(self.slider_list)
        left_widget = QWidget()
        left_widget.setLayout(self.left_layout)
        main_layout.addWidget(left_widget, alignment=Qt.AlignTop)

        # 右侧布局
        # self.right_layout = QVBoxLayout()

        # # 输入框 + 按钮
        # input_layout = QHBoxLayout()
        # self.input_field = QLineEdit(self)
        # self.add_button = QPushButton('添加', self)
        # self.add_button.clicked.connect(self.add_button_to_right_layout)
        # input_layout.addWidget(self.input_field)
        # input_layout.addWidget(self.add_button)

        # self.right_layout.addLayout(input_layout)

        # # 滚动区域
        # self.scroll_area = QScrollArea()
        # self.scroll_area.setWidgetResizable(True)

        # self.scroll_widget = QWidget()
        # self.scroll_layout = QGridLayout()
        # self.scroll_layout.setContentsMargins(10, 5, 10, 10)
        # self.scroll_layout.setSpacing(10)
        # self.scroll_layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # self.scroll_widget.setLayout(self.scroll_layout)
        # self.scroll_area.setWidget(self.scroll_widget)

        # self.right_layout.addWidget(self.scroll_area)
        # main_layout.addLayout(self.right_layout)
        # self.handle_button_click.connect(self.on_button_clicked)

        


    def init_hand(self):
        if self.hand_joint == "L25":
            # L25
            self.init_pos = [255] * 25
            # topic
            self.joint_name = ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

        elif self.hand_joint == "L21":
            # L25
            self.init_pos = [255] * 25
            # topic
            self.joint_name = ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

        elif self.hand_joint == "L20":
            self.init_pos = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
            # L20
            self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]
        elif self.hand_joint == "L10":
            # L10
            self.init_pos = [255] * 10
            self.joint_name = ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]
        elif self.hand_joint == "L7":
            # L7
            self.init_pos = [250] * 7
            self.joint_name = ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]
        self.slider_list = self.joint_name

    def on_button_clicked(self,text):
        print("_-" * 20, flush=True)
        print(text, flush=True)

    def create_sliders(self, slider_names):
        for name in slider_names:
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 255)
            slider.setValue(255)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(10)

            label = QLabel(f"{name} 0:")
            self.slider_labels[slider] = (label, name)

            slider.valueChanged.connect(self.slider_value_changed)

            h_layout = QHBoxLayout()
            h_layout.addWidget(label)
            h_layout.addWidget(slider)

            item_widget = QWidget()
            item_widget.setLayout(h_layout)

            self.left_layout.addWidget(item_widget)
            self.control_sliders.append(slider)

    def slider_value_changed(self):
        all_values = [int(slider.value()) for slider in self.control_sliders]

        sender_slider = self.sender()
        if sender_slider in self.slider_labels:
            label, name = self.slider_labels[sender_slider]
            label.setText(f"{name} {sender_slider.value()}:")

        self.last_msg = self.joint_state_msg(all_values)
        
    def loop_pub(self):
        self.thread_get_state = threading.Thread(target=self.pub_msg)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()
    def pub_msg(self):
        while True:
            start = time.time()
            self.node.publish_control_cmd(self.last_msg)
            elapsed = time.time() - start
            time.sleep(max(0, self.interval - elapsed))


    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state

    def add_button_to_right_layout(self):
        text = self.input_field.text().strip()
        if text:
            button = QPushButton(text)
            button.setFixedSize(100, 30)
            button.clicked.connect(lambda checked, text=text: self.handle_button_click.emit(text))

            # 添加到网格布局
            self.scroll_layout.addWidget(button, self.row, self.column, alignment=Qt.AlignLeft | Qt.AlignTop)

            # 更新行列位置
            self.column += 1
            if self.column >= self.BUTTONS_PER_ROW:
                self.column = 0
                self.row += 1

            self.input_field.clear()
            self.buttons.append(button)
            self.add_button_handle.emit(text)

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    time.sleep(1)
    hand_type,hand_joint,hz,is_touch = node.get_hand()
    

    app = QApplication(sys.argv)
    gui = GuiApp(hand_type=hand_type,hand_joint=hand_joint,hz=hz)
    gui.node = node
    time.sleep(1)
    gui.loop_pub()
    gui.show()

    sys.exit(app.exec_())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
