#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import numpy as np

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower', anonymous=True)
        
        # 카메라 관련 파라미터
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.camera_center = self.image_width / 2
        self.deadzone = self.image_width * 0.05  # 중앙 5% 영역은 데드존으로 설정
        
        # 제어 속도 제한
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.base_speed = 0.50  # 기본 선속도
        self.min_linear_speed = 0.4  # 최소 선속도
        self.max_linear_speed = 0.7  # 최대 선속도 (기존 base_speed)
        
        # 거리 추정을 위한 y좌표 임계값
        self.y_near = self.image_height * 0.7  # 화면 하단 70%
        self.y_far = self.image_height * 0.3   # 화면 상단 30%
        
        # Subscriber와 Publisher 설정
        self.bbox_sub = rospy.Subscriber('/bbox_center', Point, self.bbox_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 마지막 검출 시간 저장
        self.last_detection_time = rospy.Time.now()
        self.detection_timeout = rospy.Duration(2)
        
        self.tracking_enabled = False
        
        # Subscriber 추가
        self.tracking_enable_sub = rospy.Subscriber('/tracking_enable', Bool, self.tracking_enable_callback)
        
        # 마지막 제어 명령 저장
        self.last_twist = Twist()

    def bbox_callback(self, msg):
        # 트래킹이 비활성화된 상태면 무시
        if not self.tracking_enabled:
            return
            
        self.last_detection_time = rospy.Time.now()
        
        # 중앙으로부터의 오차 계산 (좌우)
        error = msg.x - self.camera_center
        
        # 데드존 체크
        if abs(error) < self.deadzone:
            angular_z = 0.0
        else:
            # 오차에 비례하는 각속도 계산
            # error를 -1 ~ 1 범위로 정규화
            normalized_error = error / self.camera_center  # -1 ~ +1
            angular_z = np.clip(normalized_error * self.max_angular_speed * 0.8, 
                    -self.max_angular_speed, self.max_angular_speed)

            
            # 각속도 제한
            angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
        
        # y좌표 기반 거리 추정 및 선속도 계산
        if msg.y > self.y_near:  # 너무 가까움
            linear_x = 0.0  # 정지
        elif msg.y < self.y_far:  # 너무 멈
            linear_x = self.max_linear_speed  # 최대 속도
        else:
            # y좌표에 따라 선형적으로 속도 조절
            y_ratio = (msg.y - self.y_far) / (self.y_near - self.y_far)
            linear_x = self.max_linear_speed - (y_ratio * 
                (self.max_linear_speed - self.min_linear_speed))
        
        # 제어 명령 생성
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        # 마지막 제어 명령 저장 및 발행
        self.last_twist = twist
        self.cmd_vel_pub.publish(twist)

    def tracking_enable_callback(self, msg):
        self.tracking_enabled = msg.data

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            time_since_last_detection = current_time - self.last_detection_time
            
            # 타임아웃 전에는 마지막 제어 명령 유지
            if self.tracking_enabled:
                if time_since_last_detection > self.detection_timeout:
                    # 타임아웃 시 정지
                    stop_twist = Twist()
                    self.cmd_vel_pub.publish(stop_twist)
                else:
                    # 마지막 제어 명령 재발행
                    self.cmd_vel_pub.publish(self.last_twist)
                    
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass