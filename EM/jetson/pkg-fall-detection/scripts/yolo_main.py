#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import asyncio
from yolo_pkg.fall_detector import EnhancedFallDetector

FRAME_STEP = 5

async def main():
    detector = EnhancedFallDetector()

    # 현재 스크립트의 절대 경로를 사용해 test_video 폴더의 파일 절대 경로 생성
    script_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_root = os.path.abspath(os.path.join(script_dir, ".."))
    video_path = os.path.join(pkg_root, "test_video", "test1.mp4")
    # video_path = 0  # 웹캠 사용

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("카메라 열기 실패!")
        return

    frame_count = 0

    # 창 크기를 조정할 수 있도록 창 생성
    cv2.namedWindow("Fall Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Fall Detection", 640, 360)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("동영상 프레임 읽기 실패!")
                break

            if frame_count % FRAME_STEP == 0:
                processed_frame, fall_detected, debug_info, *_ = await detector.process_frame(frame)
            else:
                processed_frame = frame
                fall_detected = False
                debug_info = {}

            frame_count += 1

            cv2.imshow("Fall Detection", processed_frame)
            if cv2.waitKey(60) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    asyncio.run(main())
