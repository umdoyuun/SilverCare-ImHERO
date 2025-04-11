#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import torch
from typing import Tuple, Dict
from yolo_pkg.motion_context import MotionContext

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
        left_knee_angle = self._calculate_angle(keypoints[11], keypoints[13], keypoints[15])
        right_knee_angle = self._calculate_angle(keypoints[12], keypoints[14], keypoints[16])
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
        return (velocity_score * 0.3 + acceleration_score * 0.3 + stability_score * 0.2 + activity_score * 0.2)

    def _calculate_angle(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
        v1 = p1[:2] - p2[:2]
        v2 = p3[:2] - p2[:2]
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
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