#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Header
from sensor_msgs.msg import JointState
import std_msgs.msg
from builtin_interfaces.msg import Time

import threading
import time
import signal
import sys

show_count=0
show_count_obj=0
show_step=0
joint_state = JointState() 
hand = {"joint1":255,   #拇指根部弯曲
        "joint2":128,   #拇指侧摆
        "joint3":255,   #食指根部弯曲  
        "joint4":255,   #中指根部弯曲
        "joint5":255,   #无名指根部弯曲
        "joint6":255,   #小指根部弯曲
        "joint7":255,  #拇指旋转
        }

def main(args=None):
    rclpy.init(args=args)
    node = Node("dong_test_sender")
    global show_step
    rate = 1.0 / 30  # 60 FPS
    #pub = node.create_publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)
    pub = node.create_publisher(JointState, '/cb_right_hand_control_cmd', 10)
    now = node.get_clock().now()
    
    joint_state.header = Header()

    joint_state.header.stamp = Time(sec=int(now.nanoseconds // 1e9), 
                            nanosec=int(now.nanoseconds % 1e9))
    joint_state.name=list(hand.keys())
    joint_state.velocity = [0] * len(joint_state.position)  # 与position数组长度相同，全部填充为0
    joint_state.effort = [0] * len(joint_state.position)  # 为每个关节设置努力为零
    pub.publish(joint_state)
    count = 0
    while rclpy.ok():  # 持续1秒
        position =show_left()
        if(position is not None):
            joint_state.position = position
        # rospy.loginfo(f"Publishing joint states {joint_state.__str__}")
        pub.publish(joint_state)
        time.sleep(rate)
        count = count + 1
        if show_step==54:
            show_step=0
        print(count)


def show_left():
    global show_count
    global show_count_obj
    global show_step
    global hand
    show_count= show_count+1
    if(show_count>=show_count_obj):
        show_count=0
        if(show_step==0):
            show_step=show_step+1
            show_count_obj = 100
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==1): #// 收小指与无名指
            show_step=show_step+1
            show_count_obj = 10
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint5'] = 0
            hand['joint6'] = 0
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==2): #// 将拇指搭到小指与无名指上面
            show_step=show_step+8
            show_count_obj = 30
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint7'] = 80
            return list(hand.values())
        elif(show_step==3): #// 食指和中指向一侧倾斜
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==4): #// 另一侧
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==5): #//  两支回中
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==6): #// 食指和中指做Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==7): #// 收Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==8): #// 食指和中指做Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==9): #// 收Y
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==10): #// 中指和食指弯曲伸直交替两遍
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==11): #// 中指和食指弯曲伸直交替两遍
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==12): #// 中指和食指弯曲伸直交替两遍
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==13): #// 中指和食指弯曲伸直交替两遍
            show_step=show_step+1
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==14): #// 蜷曲拇指
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint7'] = 120
            return list(hand.values())
        elif(show_step==15): #// 拇指收于掌内
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==16): #// 收4指
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 10
            hand['joint4'] = 10
            hand['joint5'] = 10
            hand['joint6'] = 10
            return list(hand.values())
        elif(show_step==17): #// 依次放开4指和拇指
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==18): #// 1
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==19): #// 2
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==20): #// 3
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            return list(hand.values())
        elif(show_step==21): #// 40
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 110
            hand['joint7'] = 240
            return list(hand.values())
        elif(show_step==22): #// 并拢拇指
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 10
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==23): #// 反转拇指指掌心
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            hand['joint2'] = 10
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==24): #// 分两步回到初始位置
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 0
            hand['joint2'] = 240
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==25): #// 1
            show_step=show_step+4
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint7'] = 110
            return list(hand.values())
        elif(show_step==26): #// 2
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==27): #// 3
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==28): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==29): #// 依次蜷曲4小指
            show_step=show_step+4
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==30): #// 蜷曲4指
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==31): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==32): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==33): #// 依次蜷曲4小指
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 0
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==34): #// 依次蜷曲4小指
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 0
            return list(hand.values())
        elif(show_step==35): #// 依次蜷曲4小指
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 0
            return list(hand.values())
        elif(show_step==36): #// 依次蜷曲4小指
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 0
            return list(hand.values())
        elif(show_step==37): #// 蜷曲拇指
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            return list(hand.values())
        elif(show_step==38): #// 打开食指和小指
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 250
            hand['joint2'] = 230
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==39): #// 打开食指和小指
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 250
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==40): #// 将拇指搭上666
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 10
            hand['joint2'] = 40
            hand['joint7'] = 60
            return list(hand.values())
        elif(show_step==41): #// 左右动手指
            show_step=show_step+5
            show_count_obj = 5
            hand['joint1'] = 50
            return list(hand.values())
        elif(show_step==42): #// 左右动手指
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==43): #// 左右动手指
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==44): #// 左右动手指
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==45): #// 左右动手指
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==46): #//  展开
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        elif(show_step==47): #// 拇指和食指捏
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 120 #155
            hand['joint2'] = 130 #130
            hand['joint3'] = 155 #158
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 90
            return list(hand.values())
        elif(show_step==48): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint3'] = 250
            return list(hand.values())
        elif(show_step==49): #// 拇指和中指捏
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 120
            hand['joint4'] = 140
            hand['joint7'] = 60
            return list(hand.values())
        elif(show_step==50): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==51): #// 拇指和无名指捏
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 120
            hand['joint2'] = 125
            hand['joint5'] = 145
            hand['joint7'] = 40
            return list(hand.values())
        elif(show_step==52): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==53): #// 拇指和小指捏
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 120
            hand['joint6'] = 135
            hand['joint7'] = 15
            return list(hand.values())
        elif(show_step==54): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 250
            return list(hand.values())
        # elif(show_step==55): #// 拇指和小指掐
        #     show_step=show_step+1
        #     show_count_obj = 40
        #     return[160,250,250,250,160, 60,128,128,128,128,50, 0, 0, 0, 0,100,250,250, 250, 80]
        # elif(show_step==56): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 20
        #     return[250,250,250,250,250,130,128,128,128,128,100, 0, 0, 0, 0,250,250,250,  50, 250]
        # elif(show_step==57): #// 拇指和无名指掐
        #     show_step=show_step+1
        #     show_count_obj = 35
        #     return[160,250,250,150,250,100,128,128,128,128,50, 0, 0, 0, 0,100,250,250, 80, 250]
        # elif(show_step==58): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 20
        #     return[250,250,250,250,250,180,128,128,128,128,100, 0, 0, 0, 0, 250,250, 50,  250, 250]
        # elif(show_step==59): #// 拇指和中指掐
        #     show_step=show_step+1
        #     show_count_obj = 35
        #     return[160,250,150,250,250,135,128,128,128,128,70, 0, 0, 0, 0,100,250,85, 250, 250]
        # elif(show_step==60): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 20
        #     return[250,250,250,250,250,220,128,128,128,128,100, 0, 0, 0, 0, 250, 50,250,  250, 250]
        # elif(show_step==61): #// 拇指和食指掐
        #     show_step=show_step+1
        #     show_count_obj = 35
        #     return[165,150,250,250,250,170,128,128,128,128,70, 0, 0, 0, 0,100,80,250, 250, 250]
        # elif(show_step==62): #// 1
        #     show_step=show_step+1
        #     show_count_obj = 60
        #     return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250,250,  250, 250]
        # else:
        #     show_step=0

def signal_handler(sig, frame):

    print('You pressed Ctrl+C!')

    sys.exit(0)  # 0表示正常退出
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    try:
        print("测试中")
        main()
    except KeyboardInterrupt:
         print("Caught KeyboardInterrupt, exiting gracefully.")
