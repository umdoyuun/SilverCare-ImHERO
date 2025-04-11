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
        self.notification_cooldown = 60
        self.camera_enabled = True
        self.family_id = None
        self.session_id = None
        try:
            self.model = YOLO(self.config.model_path)
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model.to(self.device)
            print(f"모델 로드 완료 (Device: {self.device})")
        except Exception as e:
            print(f"모델 로드 실패: {e}")
            raise
        self.fps_tracker = FPSTracker()

    def set_camera_status(self, status):
        """카메라 상태를 설정하는 메서드"""
        self.camera_enabled = status
        
    def set_family_id(self, family_id):
        """family_id를 설정하는 메서드"""
        self.family_id = family_id
        
    def set_session_id(self, session_id):
        """session_id를 설정하는 메서드"""
        self.session_id = session_id

    async def process_frame(self, frame):
        original_frame = frame.copy()
        processed_frame = frame.copy()
        fall_detected = False
        debug_info = {}
        center_point = None
        
        try:
            results = self.model(frame, verbose=False)
            
            for result in results:
                if result.keypoints is None or len(result.boxes) == 0:
                    continue
                    
                # 가장 큰 바운딩 박스를 찾습니다
                boxes = result.boxes
                areas = []
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    areas.append(area)
                
                # 가장 큰 바운딩 박스의 인덱스
                largest_idx = areas.index(max(areas))
                
                # 가장 큰 바운딩 박스의 중심점 계산
                x1, y1, x2, y2 = map(int, boxes[largest_idx].xyxy[0])
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                center_point = (center_x, center_y)
                
                # 해당하는 키포인트만 처리
                keypoints = result.keypoints.data
                if largest_idx < len(keypoints):
                    kps = keypoints[largest_idx]
                    is_fallen, person_debug_info = self.pose_analyzer.analyze_pose(kps)
                    debug_info['person'] = person_debug_info

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
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1, self.config.colors['fps_text'], 2
            )

            ret = (processed_frame, fall_detected, debug_info, center_point)
            return ret

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
        current_time = datetime.now()

        if (self.last_fall_notification_time and 
            (current_time - self.last_fall_notification_time).total_seconds() < self.notification_cooldown):
            print("알림 쿨다운 중...")
            return

        try:
            image_url = None
            if self.camera_enabled:  # 카메라가 활성화된 경우에만 이미지 처리
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                image_path = os.path.join(self.config.save_dir, f'fall_{timestamp}.jpg')
                cv2.imwrite(image_path, frame)
                
                try:
                    async with aiohttp.ClientSession() as session:
                        headers = {
                            'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                        }
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
                            if response.status == 201:
                                upload_result = await response.json()
                                image_url = upload_result['result']['file_path']
                                print(f"이미지 업로드 성공 - URL: {image_url}")
                            else:
                                print(f"이미지 업로드 실패: {response.status}")
                finally:
                    if os.path.exists(image_path):
                        os.remove(image_path)

            # 알림 API 호출 (이미지 유무와 관계없이)
            headers = {
                'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
            }
            
            notification_data = {
                "family_id": self.family_id,
                "notification_grade": "CRIT",
                "descriptions": "낙상감지"
            }

            # 이미지 URL이 있는 경우에만 추가
            if image_url:
                notification_data["image_url"] = image_url

            async with aiohttp.ClientSession() as session:
                async with session.post(
                    'https://dev-api.itdice.net/notify',
                    json=notification_data,
                    headers=headers,
                    timeout=aiohttp.ClientTimeout(total=5)
                ) as notify_response:
                    if notify_response.status == 201:
                        self.last_fall_notification_time = current_time
                        print(f"낙상 알림 전송 성공: {datetime.now().strftime('%Y%m%d_%H%M%S')}")
                    else:
                        print(f"낙상 알림 전송 실패: {notify_response.status}")

        except Exception as e:
            print("낙상 처리 중 오류 발생:", e)