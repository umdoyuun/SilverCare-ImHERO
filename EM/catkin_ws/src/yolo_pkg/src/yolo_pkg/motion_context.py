#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from collections import deque
import numpy as np
import torch
from typing import Tuple, Dict
from yolo_pkg.config import Pose, Location

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
            return float(np.mean(values))
        except Exception:
            return 0.0

    def _calculate_hip_center(self, keypoints: np.ndarray) -> Tuple[float, float]:
        # 충분한 keypoints가 없으면 기본값 반환
        if len(keypoints) < 13:
            return (0.0, 0.0)
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
        return 1.0 / (1.0 + instability)