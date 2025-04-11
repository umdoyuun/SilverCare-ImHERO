#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np
from ultralytics import YOLO
import time
import warnings
import os
from datetime import datetime
import aiohttp
import asyncio
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Dict

warnings.filterwarnings('ignore')

@dataclass
class Pose:
    keypoints: np.ndarray
    confidence: float
    timestamp: float

@dataclass
class Location:
    x: float
    y: float
    timestamp: float

class Config:
    """설정 관리 클래스"""
    def __init__(self):
        self.model_path = 'yolov8n-pose.pt'
        self.confidence_threshold = 0.4
        self.save_dir = 'fall_detection_logs'
        self.api_url = 'http://localhost:8000/fall-alert'
        self.display_size = (960, 540)  # 너무 화면이 작으면 올려올려 
        self.skeleton_connections = [
            (0,1), (0,2), (1,3), (2,4),  # 얼굴
            (5,6), (5,7), (7,9), (6,8), (8,10),  # 팔
            (5,11), (6,12), (11,12),  # 몸통
            (11,13), (13,15), (12,14), (14,16)  # 다리
        ]
        self.colors = {
            'skeleton_normal': (0, 255, 0),    # 초록색
            'skeleton_fall': (0, 0, 255),      # 빨간색
            'keypoint': (255, 255, 0),         # 노란색
            'text': (255, 255, 255),           # 흰색
            'fall_text': (0, 0, 255),          # 빨간색
            'fps_text': (0, 255, 0)            # 초록색
        }
        
        os.makedirs(self.save_dir, exist_ok=True)

class MotionContext:
    """움직임 컨텍스트 관리 클래스"""
    def __init__(self, history_size: int = 30):
        self.pose_history = deque(maxlen=history_size)
        self.location_history = deque(maxlen=history_size)
        self.velocity_history = deque(maxlen=history_size)
        self.activity_history = deque(maxlen=history_size)

    def update(self, keypoints: np.ndarray, timestamp: float):
        current_pose = Pose(keypoints=keypoints, 
                          confidence=self._calculate_confidence(keypoints),
                          timestamp=timestamp)
        self.pose_history.append(current_pose)
        
        hip_center = self._calculate_hip_center(keypoints)
        current_location = Location(hip_center[0], hip_center[1], timestamp)
        self.location_history.append(current_location)
        
        if len(self.location_history) >= 2:
            velocity = self._calculate_velocity(self.location_history[-2], current_location)
            self.velocity_history.append(velocity)
        
        current_activity = self._classify_activity()
        self.activity_history.append(current_activity)

    def _calculate_confidence(self, keypoints: np.ndarray) -> float:
        try:
            if torch.is_tensor(keypoints):
                keypoints = keypoints.cpu().numpy()
                
            values = [kp[2] for kp in keypoints]
            confidence = float(np.mean(values))
            return confidence
        except Exception as e:
            return 0.0

    def _calculate_hip_center(self, keypoints: np.ndarray) -> Tuple[float, float]:
        left_hip = keypoints[11][:2]
        right_hip = keypoints[12][:2]
        return ((left_hip[0] + right_hip[0]) / 2,
                (left_hip[1] + right_hip[1]) / 2)

    def _calculate_velocity(self, prev_loc: Location, curr_loc: Location) -> float:
        dt = curr_loc.timestamp - prev_loc.timestamp
        if dt == 0:
            return 0.0
        
        dx = curr_loc.x - prev_loc.x
        dy = curr_loc.y - prev_loc.y
        distance = np.sqrt(dx**2 + dy**2)
        return distance / dt

    def _classify_activity(self) -> str:
        if len(self.velocity_history) < 5:
            return "unknown"
            
        recent_velocity = list(self.velocity_history)[-5:]
        avg_velocity = np.mean(recent_velocity)
        
        if avg_velocity < 0.1:
            return "stationary"
        elif avg_velocity < 0.5:
            return "slow_movement"
        elif avg_velocity < 2.0:
            return "walking"
        else:
            return "rapid_movement"

    def get_motion_features(self) -> Dict:
        if len(self.velocity_history) < 2:
            return {
                "avg_velocity": 0.0,
                "acceleration": 0.0,
                "activity_type": "unknown",
                "pose_stability": 0.0
            }

        recent_velocity = list(self.velocity_history)[-5:] if len(self.velocity_history) >= 5 else list(self.velocity_history)
        
        acceleration = (self.velocity_history[-1] - self.velocity_history[-2]) / (
            self.location_history[-1].timestamp - self.location_history[-2].timestamp
        )

        return {
            "avg_velocity": np.mean(recent_velocity),
            "acceleration": acceleration,
            "activity_type": self.activity_history[-1],
            "pose_stability": self._calculate_pose_stability()
        }

    def _calculate_pose_stability(self) -> float:
        if len(self.pose_history) < 5:
            return 1.0

        recent_poses = list(self.pose_history)[-5:]
        hip_positions = [self._calculate_hip_center(pose.keypoints) for pose in recent_poses]
        
        x_std = np.std([pos[0] for pos in hip_positions])
        y_std = np.std([pos[1] for pos in hip_positions])
        
        instability = (x_std + y_std) / 2
        stability = 1.0 / (1.0 + instability)
        return stability

class EnhancedPoseAnalyzer:
    """포즈 분석기"""
    def __init__(self):
        self.motion_context = MotionContext()
        self.min_confidence = 0.4
        self.fall_duration_threshold = 2.0
        self.fall_start_time = None
        self.fall_status = False
        
        self.weights = {
            'vertical': 0.3,
            'posture': 0.2,
            'angles': 0.2,
            'motion': 0.3
        }

    def analyze_pose(self, keypoints: np.ndarray) -> Tuple[bool, Dict]:
            if torch.is_tensor(keypoints):
                keypoints = keypoints.cpu().numpy()
                
            current_time = time.time()
            self.motion_context.update(keypoints, current_time)
            
            vertical_score = self._analyze_vertical_alignment(keypoints)
            posture_score = self._analyze_posture(keypoints)
            angle_score = self._analyze_joint_angles(keypoints)
            
            motion_features = self.motion_context.get_motion_features()
            motion_score = self._analyze_motion_context(motion_features)
            
            fall_score = (
                vertical_score * self.weights['vertical'] +
                posture_score * self.weights['posture'] +
                angle_score * self.weights['angles'] +
                motion_score * self.weights['motion']
            )

            current_fall_state = fall_score > 0.7
            is_fall_detected = self._evaluate_fall_state(current_fall_state, current_time)
            
            debug_info = {
                'fall_score': fall_score,
                'vertical_score': vertical_score,
                'posture_score': posture_score,
                'angle_score': angle_score,
                'motion_score': motion_score,
                'motion_features': motion_features,
                'is_fallen': is_fall_detected
            }
            
            return is_fall_detected, debug_info

    def _analyze_vertical_alignment(self, keypoints: np.ndarray) -> float:
        y_coords = {
            'shoulder': (keypoints[5][1] + keypoints[6][1]) / 2,
            'hip': (keypoints[11][1] + keypoints[12][1]) / 2,
            'knee': (keypoints[13][1] + keypoints[14][1]) / 2,
            'ankle': (keypoints[15][1] + keypoints[16][1]) / 2
        }
        
        is_vertical = (y_coords['shoulder'] < y_coords['hip'] < 
                      y_coords['knee'] < y_coords['ankle'])
                      
        height_diffs = [abs(y2 - y1) for y1, y2 in zip(
            list(y_coords.values())[:-1], 
            list(y_coords.values())[1:]
        )]
        is_horizontal = all(diff < 20 for diff in height_diffs)
        
        if is_horizontal:
            return 1.0
        return 0.0 if is_vertical else 0.8

    def _analyze_posture(self, keypoints: np.ndarray) -> float:
        shoulder_center = (keypoints[5][:2] + keypoints[6][:2]) / 2
        hip_center = (keypoints[11][:2] + keypoints[12][:2]) / 2
        
        dx = hip_center[0] - shoulder_center[0]
        dy = hip_center[1] - shoulder_center[1]
        
        if dy == 0:
            angle = 90
        else:
            angle = abs(np.degrees(np.arctan2(dx, dy)))
            
        return min(1.0, angle / 45.0)

    def _analyze_joint_angles(self, keypoints: np.ndarray) -> float:
        left_knee_angle = self._calculate_angle(
            keypoints[11], keypoints[13], keypoints[15]
        )
        right_knee_angle = self._calculate_angle(
            keypoints[12], keypoints[14], keypoints[16]
        )
        
        left_score = 1.0 - max(0, min(1.0, (left_knee_angle - 90) / 90))
        right_score = 1.0 - max(0, min(1.0, (right_knee_angle - 90) / 90))
        
        return max(left_score, right_score)

    def _analyze_motion_context(self, motion_features: Dict) -> float:
        velocity_score = min(1.0, motion_features['avg_velocity'] / 3.0)
        acceleration_score = min(1.0, abs(motion_features['acceleration']) / 5.0)
        stability_score = 1.0 - motion_features['pose_stability']
        
        activity_weights = {
            'stationary': 0.3,
            'slow_movement': 0.5,
            'walking': 0.7,
            'rapid_movement': 1.0,
            'unknown': 0.5
        }
        
        activity_score = activity_weights[motion_features['activity_type']]
        
        return (velocity_score * 0.3 + 
                acceleration_score * 0.3 + 
                stability_score * 0.2 + 
                activity_score * 0.2)

    def _calculate_angle(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
        v1 = p1[:2] - p2[:2]
        v2 = p3[:2] - p2[:2]
        
        cos_angle = np.dot(v1, v2) / (
            np.linalg.norm(v1) * np.linalg.norm(v2)
        )
        
        return np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

    def _evaluate_fall_state(self, current_fall_state: bool, current_time: float) -> bool:
        if current_fall_state and not self.fall_status:
            self.fall_status = True
            self.fall_start_time = current_time
            return False
            
        elif current_fall_state and self.fall_status:
            duration = current_time - self.fall_start_time
            return duration >= self.fall_duration_threshold
            
        else:
            self.fall_status = False
            self.fall_start_time = None
            return False

class FPSTracker:
    """FPS 추적 클래스"""
    def __init__(self, avg_frames=30):
        self.avg_frames = avg_frames
        self.frame_times = []
        self.last_time = time.time()
    
    def update(self):
        current_time = time.time()
        self.frame_times.append(current_time - self.last_time)
        self.last_time = current_time
        
        if len(self.frame_times) > self.avg_frames:
            self.frame_times.pop(0)
        
        avg_time = sum(self.frame_times) / len(self.frame_times)
        return 1.0 / avg_time if avg_time > 0 else 0

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

        try:
            results = self.model(frame, verbose=False)
            
            for result in results:
                if result.keypoints is None:
                    continue
                    
                keypoints = result.keypoints.data
                for person_idx, kps in enumerate(keypoints):
                    is_fallen, person_debug_info = self.pose_analyzer.analyze_pose(kps)
                    debug_info[f'person_{person_idx}'] = person_debug_info
                    
                    # 시각화
                    processed_frame = self.visualize_detection(
                        processed_frame, 
                        kps, 
                        is_fallen,
                        person_debug_info
                    )
                    
                    if is_fallen:
                        fall_detected = True
                        await self.handle_fall_detection(original_frame)

            # FPS 표시
            fps = self.fps_tracker.update()
            cv2.putText(
                processed_frame,
                f"FPS: {fps:.1f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1, self.config.colors['fps_text'], 2
            )

            return processed_frame, fall_detected, debug_info

        except Exception as e:
            return frame, False, {}

    def visualize_detection(self, frame, keypoints, is_fallen, debug_info):
        """감지 결과 시각화"""
        try:
            # 스켈레톤 그리기
            color = self.config.colors['skeleton_fall'] if is_fallen else self.config.colors['skeleton_normal']
            
            for connection in self.config.skeleton_connections:
                pt1 = tuple(map(int, keypoints[connection[0]][:2]))
                pt2 = tuple(map(int, keypoints[connection[1]][:2]))
                cv2.line(frame, pt1, pt2, color, 2)
            
            # 키포인트 그리기
            for kp in keypoints:
                x, y = map(int, kp[:2])
                cv2.circle(frame, (x, y), 3, self.config.colors['keypoint'], -1)
            
            # 낙상 상태 표시
            if is_fallen:
                cv2.putText(
                    frame,
                    "FALL DETECTED",
                    (int(frame.shape[1]/2) - 100, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, self.config.colors['fall_text'], 2
                )
            
            # 스코어 및 디버그 정보 표시
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
            print(f"시각화 오류: {e}")
            return frame

    async def handle_fall_detection(self, frame):
        """낙상 감지 처리"""
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
            print(f"낙상 처리 중 오류 발생: {e}")

async def main():
    """메인 함수"""
    config = Config()
    
    try:
        # 비디오 캡처 초기화
        video_path = "test_video/test2.mp4"
        # video_path = 0 # 웹캠 사용
        cap = cv2.VideoCapture(video_path)  
        if not cap.isOpened():
            raise IOError("Cannot open video source")
        
        # 원본 비디오의 FPS 가져오기
        original_fps = cap.get(cv2.CAP_PROP_FPS)
        if original_fps == 0:
            original_fps = 30  # 웹캠 사용시 기본값
        
        # 낙상 감지기 초기화
        detector = EnhancedFallDetector()
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임 읽기 실패")
                break
            
            # 프레임 처리
            processed_frame, fall_detected, debug_info = await detector.process_frame(frame)
            
            # 화면에 맞게 크기 조정
            display_frame = cv2.resize(
                processed_frame, 
                config.display_size,
                interpolation=cv2.INTER_LINEAR
            )
            
            # 결과 표시
            cv2.imshow('Enhanced Fall Detection', display_frame)
            
            # FPS에 맞춰 딜레이
            delay = int(1000 / original_fps)
            if cv2.waitKey(delay) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"실행 중 오류 발생: {e}")
        
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())