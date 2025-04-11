#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import Bool
M_PI = 3.141592

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')
        
        # 왼쪽 스틱만 사용
        self.axis_forward = rospy.get_param('~axis_linear', 1)   # 왼쪽 스틱 Y축
        self.axis_turn = rospy.get_param('~axis_angular', 3)     # 오른쪽 스틱 X축
        self.scale_linear = rospy.get_param('~scale_linear', 0.5)
        self.scale_angular = rospy.get_param('~scale_angular', 1.0)
        self.deadzone = 0.1  # 데드존 추가
        
        self.last_steering = 0.0  # 마지막 조향각 저장
        self.forward = 0.0        # forward 값 초기화 추가
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.loginfo("Joy Teleop Ready!")
        self.publish_rate = rospy.Rate(10)  # 10Hz로 publish
        self.manual_mode = False  # 수동 모드 플래그
        self.mode_button = rospy.get_param('~mode_button', 0)  # A버튼을 모드 전환으로 사용
        self.stop_button = rospy.get_param('~stop_button', 1)  # B버튼
        self.prev_stop_button = 0
        
        # move_base enable/disable을 위한 서비스 클라이언트
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.stop_pub = rospy.Publisher('/stop_recovery', Bool, queue_size=1)
        self.prev_mode_button = 0
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            # manual 모드일 때만 publish
            if self.manual_mode:
                twist = Twist()
                twist.linear.x = self.scale_linear * self.forward
                twist.angular.z = self.scale_angular * self.last_steering
                self.cmd_vel_pub.publish(twist)
            self.publish_rate.sleep()

    def joy_callback(self, joy_msg):
        # 모드 전환 버튼 확인
        if joy_msg.buttons[self.mode_button] == 1:
            self.manual_mode = not self.manual_mode
            if self.manual_mode:
                self.move_base_client.cancel_all_goals()
                rospy.loginfo("Manual Control Mode")
            else:
                # autonomous 모드 전환시 값 초기화
                self.forward = 0.0
                self.last_steering = 0.0
                rospy.loginfo("Autonomous Mode")

        # B버튼 상태 확인 (rising edge 검출)
        stop_button = joy_msg.buttons[self.stop_button]
        if stop_button == 1 and self.prev_stop_button == 0:
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
        
        self.prev_stop_button = stop_button
        
        # 수동 모드일 때만 조이스틱 입력 처리
        if self.manual_mode:
            twist = Twist()
            
            # 전진/후진 (왼쪽 스틱 Y축)
            forward = joy_msg.axes[self.axis_forward]
            if abs(forward) > self.deadzone:
                self.forward = forward
            else:
                self.forward = 0.0
            twist.linear.x = self.scale_linear * self.forward
            
            # 조향 (오른쪽 스틱 X축)
            turn = joy_msg.axes[self.axis_turn]
            if abs(turn) > self.deadzone:
                self.last_steering = turn
            else:
                self.last_steering = 0.0
                
            twist.angular.z = self.scale_angular * self.last_steering
            self.cmd_vel_pub.publish(twist)
        else:
            twist = Twist()
        
            # 전진/후진 (Y축) - 부호 변경
            self.forward = joy_msg.axes[self.axis_forward]
            twist.linear.x = self.scale_linear * self.forward
        
            # 조향 (X축) - 부호 변경
            turn = joy_msg.axes[self.axis_turn]
        
            # 데드존 적용하여 중립 복귀 처리
            if abs(turn) > self.deadzone:
                self.last_steering = turn
            else:
                self.last_steering = 0.0  # 중립 복귀
        
            # 정지 상태에서도 조향각 유지
            twist.angular.z = self.scale_angular * self.last_steering
        
            self.cmd_vel_pub.publish(twist)
        
if __name__ == '__main__':
    try:
        JoyTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass