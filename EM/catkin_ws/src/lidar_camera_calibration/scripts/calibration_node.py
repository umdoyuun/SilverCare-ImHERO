#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class LidarCameraCalibrator:
    def __init__(self):
        rospy.init_node('lidar_camera_calibrator')
        
        # 카메라 intrinsic 파라미터
        self.fx = rospy.get_param('~fx')
        self.cx = rospy.get_param('~cx')
        
        # LiDAR-카메라 extrinsic 파라미터
        self.x_offset = rospy.get_param('~x_offset')
        self.y_offset = rospy.get_param('~y_offset')
        self.yaw = rospy.get_param('~yaw')
        
        # tf 관련 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        
        # 현재 캘리브레이션 파라미터
        self.z_offset = rospy.get_param('~z_offset', 0.0)
        
        # 카메라 파라미터
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_center = self.image_width / 2
        
        # 시각화 파라미터
        self.marker_size = rospy.get_param('~marker_size', 0.05)
        self.marker_color_r = rospy.get_param('~marker_color_r', 1.0)
        self.marker_color_a = rospy.get_param('~marker_color_a', 1.0)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.bbox_sub = rospy.Subscriber('/bbox_center', Point, self.bbox_callback)
        
        # Publishers
        self.marker_pub = rospy.Publisher('/calibration_markers', Marker, queue_size=10)
        
        # 캘리브레이션 데이터 저장
        self.lidar_points = []
        self.camera_points = []
        
    def scan_callback(self, scan_msg):
        # LiDAR 데이터 처리
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, 
                         scan_msg.angle_max + scan_msg.angle_increment, 
                         scan_msg.angle_increment)
        
        # 유효한 포인트 추출
        valid_idx = np.where((ranges > scan_msg.range_min) & 
                           (ranges < scan_msg.range_max))[0]
        
        if len(valid_idx) > 0:
            self.visualize_lidar_points(ranges[valid_idx], 
                                      angles[valid_idx])
    
    def bbox_callback(self, msg):
        # 바운딩 박스 중심점을 3D 공간으로 투영
        normalized_x = (msg.x - 320) / 320  # 이미지 중심 기준 정규화
        
        # 현재 transform 발행
        self.publish_camera_lidar_transform()
        
    def visualize_lidar_points(self, ranges, angles):
        marker = Marker()
        marker.header.frame_id = "laser_frame"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0
        
        for r, theta in zip(ranges, angles):
            p = geometry_msgs.msg.Point()
            p.x = r * np.cos(theta)
            p.y = r * np.sin(theta)
            p.z = 0
            marker.points.append(p)
            
        self.marker_pub.publish(marker)
        
    def publish_camera_lidar_transform(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "laser_frame"
        t.child_frame_id = "camera_frame"
        
        t.transform.translation.x = self.x_offset
        t.transform.translation.y = self.y_offset
        t.transform.translation.z = self.z_offset
        
        q = tf2_ros.transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        calibrator = LidarCameraCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass