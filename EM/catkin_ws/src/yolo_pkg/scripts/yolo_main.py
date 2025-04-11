#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import asyncio
from yolo_pkg.fall_detector import EnhancedFallDetector
from geometry_msgs.msg import Point
from std_msgs.msg import String  # String 메시지 타입 추가

FRAME_STEP = 1

# 콜백 함수들
def session_callback(msg, detector):
    detector.set_session_id(msg.data)
    rospy.loginfo(f"세션 ID 설정됨: {msg.data}")

def family_callback(msg, detector):
    detector.set_family_id(msg.data)
    rospy.loginfo(f"패밀리 ID 설정됨: {msg.data}")

async def main():
    rospy.init_node('yolo_detection', anonymous=True)
    detector = EnhancedFallDetector()
    
    # 바운딩 박스 중심점 퍼블리셔 생성
    bbox_center_pub = rospy.Publisher('bbox_center', Point, queue_size=1, latch=True)

    # 세션 ID와 패밀리 ID 구독자 생성
    rospy.Subscriber('session_id', String, session_callback, callback_args=detector)
    rospy.Subscriber('family_id', String, family_callback, callback_args=detector)

    # 현재 스크립트의 절대 경로를 사용해 test_video 폴더의 파일 절대 경로 생성
    script_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_root = os.path.abspath(os.path.join(script_dir, ".."))
    video_path = os.path.join(pkg_root, "test_video", "test2.mp4")
    video_path = 0  # 웹캠 사용

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("카메라 열기 실패!")
        return

    frame_count = 0

    # 창 크기를 조정할 수 있도록 창 생성
    cv2.namedWindow("Fall Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Fall Detection", 640, 360)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("동영상 프레임 읽기 실패!")
            break

        if frame_count % FRAME_STEP == 0:
            processed_frame, fall_detected, debug_info, center_point = await detector.process_frame(frame)
            
            # center_point가 있으면 토픽 발행
            if center_point is not None:
                point_msg = Point()
                point_msg.x = center_point[0]
                point_msg.y = center_point[1]
                point_msg.z = 0.0  # z 좌표는 2D 이미지에서는 사용하지 않음
                bbox_center_pub.publish(point_msg)
        else:
            processed_frame = frame

        frame_count += 1

        cv2.imshow("Fall Detection", processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
        pass
