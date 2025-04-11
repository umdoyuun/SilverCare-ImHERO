from datetime import datetime, timezone, timedelta
import aiohttp
import logging
from typing import Optional, Dict, Any
from sqlalchemy.orm import Session

from models import Account, LocationMap

logger = logging.getLogger(__name__)

class WeatherService:
    def __init__(self, api_key: str):
        self.api_key = api_key

    async def get_weather_for_user(self, user_id: str, db: Session) -> Optional[Dict]:
        try:
            # 사용자의 주소 정보 조회
            user = db.query(Account).filter(Account.id == user_id).first()
            if not user or not user.address:
                logger.error(f"User {user_id} address not found")
                return None

            # 주소에 해당하는 좌표 조회
            location = db.query(LocationMap)\
                .filter(LocationMap.address == user.address)\
                .first()
            if not location:
                logger.error(f"Location not found for address: {user.address}")
                return None

            weather_data = await self.fetch_weather_data(location.x_value, location.y_value)
            if weather_data:
                return self._format_weather_data(weather_data, user.address)
            return None

        except Exception as e:
            logger.error(f"Weather service error: {str(e)}")
            return None

    async def fetch_weather_data(self, x: int, y: int) -> Optional[Dict]:
        try:
            base_date, base_time = self._get_base_time()
            url = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getUltraSrtFcst"
            params = {
                'serviceKey': self.api_key,
                'pageNo': '1',
                'numOfRows': '60',
                'dataType': 'JSON',
                'base_date': base_date,
                'base_time': base_time,
                'nx': str(x),
                'ny': str(y)
            }

            async with aiohttp.ClientSession() as session:
                async with session.get(url, params=params) as response:
                    if response.status == 200:
                        return await response.json()
                    logger.error(f"Weather API HTTP error: {response.status}")
                    return None
        except Exception as e:
            logger.error(f"날씨 데이터 조회 오류: {str(e)}")
            return None

    def _get_base_time(self) -> tuple[str, str]:
        now = datetime.now(timezone.utc) + timedelta(hours=9)
        if now.minute < 45:
            now = now.replace(hour=now.hour - 1)
        base_date = now.strftime("%Y%m%d")
        base_time = f"{now.hour:02d}00"
        return base_date, base_time

    def _format_weather_data(self, data: Dict, address: str) -> Dict:
        try:
            items = data.get('response', {}).get('body', {}).get('items', {}).get('item', [])
            result = {
                'address': address,
                'temperature': 'N/A',
                'sky': '알 수 없음',
                'precipitation': '없음',
                'humidity': 'N/A'
            }

            for item in items:
                category = item.get('category')
                value = item.get('fcstValue')

                if category == 'T1H':
                    result['temperature'] = f"{value}°C"
                elif category == 'SKY':
                    sky_codes = {'1': '맑음', '2': '구름조금', '3': '구름많음', '4': '흐림'}
                    result['sky'] = sky_codes.get(value, '알 수 없음')
                elif category == 'PTY':
                    rain_codes = {'0': '없음', '1': '비', '2': '비/눈', '3': '눈', '4': '소나기'}
                    result['precipitation'] = rain_codes.get(value, '없음')
                elif category == 'REH':
                    result['humidity'] = f"{value}%"

            return result
        except Exception as e:
            logger.error(f"날씨 데이터 가공 오류: {str(e)}")
            return {
                'address': address,
                'temperature': 'N/A',
                'sky': '알 수 없음',
                'precipitation': '없음',
                'humidity': 'N/A'
            }