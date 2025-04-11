from datetime import datetime
from typing import Dict, Any, Optional

class CacheManager:
    def __init__(self):
        self.weather_cache: Dict[str, Dict[str, Any]] = {}
        self.durations = {
            'weather': 3600,  # 1시간
            'disaster': 300   # 5분
        }

    def get_weather(self, user_id: str) -> Optional[Dict]:
        return self._get_from_cache(self.weather_cache, user_id, self.durations['weather'])

    def set_weather(self, user_id: str, data: Dict):
        self._set_in_cache(self.weather_cache, user_id, data)

    def _get_from_cache(self, cache: Dict, key: str, duration: int) -> Optional[Dict]:
        if key not in cache:
            return None
            
        entry = cache[key]
        age = (datetime.now() - entry['timestamp']).seconds
        
        if age > duration:
            del cache[key]
            return None
            
        return entry['data']

    def _set_in_cache(self, cache: Dict, key: str, data: Dict):
        cache[key] = {
            'data': data,
            'timestamp': datetime.now()
        }