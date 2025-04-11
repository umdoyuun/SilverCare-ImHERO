#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

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