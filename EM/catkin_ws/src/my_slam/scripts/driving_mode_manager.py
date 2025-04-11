#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy  # Joy 메시지 import 추가

class DrivingModeManager:
    def __init__(self):
        rospy.init_node('driving_mode_manager', anonymous=True)
        
        # 현재 주행 모드
        self.current_mode = "stop"
        
        # Subscribers
        self.mode_sub = rospy.Subscriber('driving_mode', String, self.mode_callback)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)  # 조이스틱 구독자 추가
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.exploration_enable_pub = rospy.Publisher('/exploration_enable', Bool, queue_size=1)
        self.tracking_enable_pub = rospy.Publisher('/tracking_enable', Bool, queue_size=1)
        
        # 버튼 상태 저장 (이전 상태와 비교하기 위함)
        self.prev_x_button = 0
        self.prev_y_button = 0
    
    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.current_mode:
            rospy.loginfo(f"주행 모드 변경: {self.current_mode} -> {new_mode}")
            self.handle_mode_change(new_mode)
            self.current_mode = new_mode
    
    def handle_mode_change(self, new_mode):
        # 모든 모드 비활성화 (로그 제거)
        self.exploration_enable_pub.publish(False)
        self.tracking_enable_pub.publish(False)
        
        # 로봇 정지
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # 새로운 모드 활성화
        rospy.sleep(0.5)  # 안정화를 위한 대기
        if new_mode == "autonomous":
            self.exploration_enable_pub.publish(True)
            rospy.loginfo("자율 주행 모드로 전환")
        elif new_mode == "tracking":
            self.tracking_enable_pub.publish(True)
            rospy.loginfo("사람 추적 모드로 전환")
        elif new_mode == "stop":
            rospy.loginfo("정지 모드로 전환")

    def joy_callback(self, msg):
        # X 버튼 (인덱스 2)로 exploration 모드 토글
        if msg.buttons[2] == 1 and self.prev_x_button == 0:
            new_mode = "autonomous" if self.current_mode == "stop" else "stop"
            self.handle_mode_change(new_mode)
            self.current_mode = new_mode
        
        # Y 버튼 (인덱스 3)으로 tracking 모드 토글
        if msg.buttons[3] == 1 and self.prev_y_button == 0:
            new_mode = "tracking" if self.current_mode == "stop" else "stop"
            self.handle_mode_change(new_mode)
            self.current_mode = new_mode
        
        # 버튼 상태 저장
        self.prev_x_button = msg.buttons[2]
        self.prev_y_button = msg.buttons[3]

if __name__ == '__main__':
    try:
        manager = DrivingModeManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass