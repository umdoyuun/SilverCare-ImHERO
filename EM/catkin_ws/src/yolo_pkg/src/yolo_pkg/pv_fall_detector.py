#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import os
import asyncio
import aiohttp
from datetime import datetime
from ultralytics import YOLO
from yolo_pkg.config import Config
from yolo_pkg.pose_analyzer import EnhancedPoseAnalyzer
from yolo_pkg.fps_tracker import FPSTracker

class EnhancedFallDetector:
    """향상된 낙상 감지 클래스"""
    def __init__(self):
        self.config = Config()
        self.pose_analyzer = EnhancedPoseAnalyzer()
        try:
            self.model = YOLO(self.config.model_path)
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model.to(self.device)
            print(f"모델 로드 완료 (Device: {self.device})")
        except Exception as e:
            print(f"모델 로드 실패: {e}")
            raise
        self.fps_tracker = FPSTracker()

    async def process_frame(self, frame):
        original_frame = frame.copy()
        processed_frame = frame.copy()
        fall_detected = False
        debug_info = {}
        center_point = None  # 바운딩 박스 중심점 초기화
        
        try:
            results = self.model(frame, verbose=False)
            for result in results:
                if result.keypoints is None:
                    continue
                
                # 첫 번째 검출된 사람의 바운딩 박스 중심점만 사용
                if len(result.boxes) > 0:
                    box = result.boxes[0]  # 첫 번째 박스
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # 중심점 계산
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    center_point = (center_x, center_y)
                
                keypoints = result.keypoints.data
                for person_idx, kps in enumerate(keypoints):
                    is_fallen, person_debug_info = self.pose_analyzer.analyze_pose(kps)
                    debug_info[f'person_{person_idx}'] = person_debug_info

                    processed_frame = self.visualize_detection(
                        processed_frame,
                        kps,
                        is_fallen,
                        person_debug_info
                    )

                    if is_fallen:
                        fall_detected = True
                        await self.handle_fall_detection(original_frame)

            fps = self.fps_tracker.update()
            cv2.putText(
                processed_frame,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, self.config.colors['fps_text'], 2
            )

            return processed_frame, fall_detected, debug_info, center_point

        except Exception as e:
            print(f"프레임 처리 중 오류 발생: {e}")
            return frame, False, {}, None

    def visualize_detection(self, frame, keypoints, is_fallen, debug_info):
        try:
            color = self.config.colors['skeleton_fall'] if is_fallen else self.config.colors['skeleton_normal']

            for connection in self.config.skeleton_connections:
                if max(connection) >= len(keypoints):
                    # keypoints 개수가 부족하면, 해당 연결은 무시
                    continue
                pt1 = tuple(map(int, keypoints[connection[0]][:2]))
                pt2 = tuple(map(int, keypoints[connection[1]][:2]))
                cv2.line(frame, pt1, pt2, color, 2)

            for kp in keypoints:
                x, y = map(int, kp[:2])
                cv2.circle(frame, (x, y), 3, self.config.colors['keypoint'], -1)

            if is_fallen:
                cv2.putText(
                    frame,
                    "FALL DETECTED",
                    (int(frame.shape[1] / 2) - 100, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, self.config.colors['fall_text'], 2
                )

            y_pos = 70
            for key, value in debug_info.items():
                if isinstance(value, (int, float)):
                    text = f"{key}: {value:.2f}"
                    cv2.putText(
                        frame, text,
                        (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, self.config.colors['text'], 1
                    )
                    y_pos += 20
                elif isinstance(value, dict):
                    for sub_key, sub_value in value.items():
                        if isinstance(sub_value, (int, float)):
                            text = f"{sub_key}: {sub_value:.2f}"
                            cv2.putText(
                                frame, text,
                                (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, self.config.colors['text'], 1
                            )
                            y_pos += 20

            return frame

        except Exception as e:
            print("시각화 오류:", e)
            return frame

    async def handle_fall_detection(self, frame):
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(self.config.save_dir, f'fall_{timestamp}.jpg')
            cv2.imwrite(image_path, frame)

            alert_data = {
                "image_path": image_path,
                "timestamp": timestamp,
                "device_id": "camera_1"
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(
                    self.config.api_url,
                    json=alert_data,
                    timeout=aiohttp.ClientTimeout(total=5)
                ) as response:
                    if response.status == 200:
                        print(f"낙상 알림 전송 성공: {timestamp}")
                    else:
                        print(f"낙상 알림 전송 실패: {response.status}")

        except Exception as e:
            print("낙상 처리 중 오류 발생:", e)