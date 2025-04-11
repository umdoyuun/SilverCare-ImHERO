#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import socket
import json
from std_msgs.msg import String, Bool

class SocketClient:
    def __init__(self):
        rospy.init_node('socket_client', anonymous=True)
        
        # 소켓 통신 설정
        self.HOST = rospy.get_param('~server_ip', '70.12.247.214')
        self.PORT = rospy.get_param('~server_port', 12345)
        
        # ROS 퍼블리셔 설정
        self.camera_pub = rospy.Publisher('camera_status', Bool, queue_size=10)
        self.driving_pub = rospy.Publisher('driving_mode', String, queue_size=10)
        self.session_pub = rospy.Publisher('session_id', String, queue_size=1, latch=True)
        self.user_id_pub = rospy.Publisher('user_id', String, queue_size=1, latch=True)
        self.family_id_pub = rospy.Publisher('family_id', String, queue_size=1, latch=True)
        
        # 현재 상태 저장
        self.current_session_id = None
        self.current_user_id = None
        self.current_family_id = None
        self.is_connected = False
        
        # 소켓 연결
        self.socket = None
        self.connect_to_server()
        
        # 주기적 publish를 위한 타이머 설정 (5초마다)
        self.publish_timer = rospy.Timer(rospy.Duration(5), self.publish_current_status)

    def publish_current_status(self, event):
        """현재 상태를 주기적으로 발행하는 콜백 함수"""
        if self.current_session_id is not None:
            self.session_pub.publish(self.current_session_id)
        
        if self.current_family_id is not None:
            self.family_id_pub.publish(self.current_family_id)

    def connect_to_server(self):
        while not rospy.is_shutdown() and not self.is_connected:
            try:
                if self.socket:
                    self.socket.close()
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.HOST, self.PORT))
                self.is_connected = True
                rospy.loginfo(f"Connected to server at {self.HOST}:{self.PORT}")
            except Exception as e:
                rospy.logerr(f"Connection failed: {e}")
                rospy.sleep(5)  # 5초 대기 후 재시도

    def process_message(self, message):
        try:
            data = json.loads(message)
            
            # user_id 처리 - 항상 publish
            if 'user_id' in data:
                self.current_user_id = data['user_id']
                self.user_id_pub.publish(self.current_user_id)
            
            # session_id 처리 - 항상 publish
            if 'session_id' in data:
                self.current_session_id = data['session_id']
                self.session_pub.publish(self.current_session_id)
            
            # family_id 처리 - 항상 publish
            if 'family_id' in data:
                self.current_family_id = data['family_id']
                self.family_id_pub.publish(self.current_family_id)
            
            # 카메라 상태 처리
            if 'is_camera_enabled' in data:
                self.camera_pub.publish(data['is_camera_enabled'])
            
            # 주행 상태 처리
            if 'is_driving_enabled' in data:
                driving_status = 'autonomous' if data['is_driving_enabled'] else 'stop'
                self.driving_pub.publish(driving_status)
                    
        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON decode error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            try:
                if not self.is_connected:
                    self.connect_to_server()
                    continue
                
                data = self.socket.recv(1024)
                if data:
                    message = data.decode()
                    rospy.loginfo(f"Received raw message: {message}")
                    self.process_message(message)
                
            except Exception as e:
                rospy.logerr(f"Connection error: {e}")
                self.is_connected = False
                continue
                
            rate.sleep()

        if self.socket:
            self.socket.close()

    def __del__(self):
        """소멸자: 타이머 정리"""
        if hasattr(self, 'publish_timer'):
            self.publish_timer.shutdown()

if __name__ == '__main__':
    try:
        client = SocketClient()
        client.run()
    except rospy.ROSInterruptException:
        pass