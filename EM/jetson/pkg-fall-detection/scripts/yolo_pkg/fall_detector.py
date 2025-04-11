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
    """향상된 낙상 감지 클래스 + 쿨다운 추가"""
    def __init__(self):
        self.config = Config()
        self.pose_analyzer = EnhancedPoseAnalyzer()
        self.last_fall_notification_time = None
        self.notification_colldown = 60
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
        
        try:
            results = self.model(frame, verbose=False)
            
            print("YOLO results type:", type(results))
            results_list = list(results)  
            print("YOLO results contents:", results_list)
            
            for result in results:
                if result.keypoints is None:
                    continue
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

            # 튜플로 묶어서 반환
            ret = (processed_frame, fall_detected, debug_info)
            print("반환값 타입:", type(ret))
            print("반환값 길이:", len(ret))
            return ret

        except Exception as e:
            print(f"프레임 처리 중 오류 발생: {e}")
            return (frame, False, {})  

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
        current_time = datetime.now()

        if (self.last_fall_notification_time and 
            (current_time - self.last_fall_notification_time).total_seconds() < self.notification_cooldown):
            print("알림 쿨다운 중...")
            return

        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(self.config.save_dir, f'fall_{timestamp}.jpg')
            cv2.imwrite(image_path, frame)
            
            headers = {
                'Cookie': "session_id=60be008afcfbaae56e289f6c243eb6d8; Path=/; Domain=itdice.net; Secure; HttpOnly;"
            }
            
            async with aiohttp.ClientSession() as session:
                # 1. 이미지 업로드
                data = aiohttp.FormData()
                data.add_field('file',
                            open(image_path, 'rb'),
                            filename=f'fall_{timestamp}.jpg',
                            content_type='image/jpeg')
                
                async with session.post(
                    'https://image.itdice.net/upload',
                    data=data,
                    headers=headers,
                    timeout=aiohttp.ClientTimeout(total=30)
                ) as response:
                    if response.status != 201:
                        print(f"이미지 업로드 실패: {response.status}")
                        return
                        
                    upload_result = await response.json()
                    image_url = upload_result['result']['file_path']
                    print(f"이미지 업로드 성공 - URL: {image_url}")
                    
                # 2. 알림 API 호출
                notification_data = {
                    "family_id": "FlcuDLxVC9SolW70",
                    "notification_grade": "CRIT",
                    "descriptions": "낙상감지",
                    "image_url": image_url
                }
                
                async with session.post(
                    'https://dev-api.itdice.net/notify',
                    json=notification_data,
                    headers=headers,
                    timeout=aiohttp.ClientTimeout(total=5)
                ) as notify_response:
                    if notify_response.status == 201:
                        self.last_fall_notification_time = current_time
                        print(f"낙상 알림 전송 성공: {timestamp}")
                    else:
                        print(f"낙상 알림 전송 실패: {notify_response.status}")

        except Exception as e:
            print("낙상 처리 중 오류 발생:", e)
        finally:
            if os.path.exists(image_path):
                os.remove(image_path)