#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import warnings
import numpy as np
import torch
from dataclasses import dataclass

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
        self.api_url = 'http://dev-api.itdice.net/notify'
        self.display_size = (960, 540)
        self.skeleton_connections = [
            (0,1), (0,2), (1,3), (2,4),  # 얼굴
            (5,6), (5,7), (7,9), (6,8), (8,10),  # 팔
            (5,11), (6,12), (11,12),  # 몸통
            (11,13), (13,15), (12,14), (14,16)  # 다리
        ]
        self.colors = {
            'skeleton_normal': (0, 255, 0),
            'skeleton_fall': (0, 0, 255),
            'keypoint': (255, 255, 0),
            'text': (255, 255, 255),
            'fall_text': (0, 0, 255),
            'fps_text': (0, 255, 0)
        }
        
        os.makedirs(self.save_dir, exist_ok=True)