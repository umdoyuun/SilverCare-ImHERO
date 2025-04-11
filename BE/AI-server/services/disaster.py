import logging
from datetime import datetime
import requests
from typing import Dict, List
from sqlalchemy.orm import Session
import json

from models import Account, Family, Notification

from utils.timezone_utils import get_kst_today

logger = logging.getLogger(__name__)

class DisasterService:
    def __init__(self, api_key: str):
        self.api_key = api_key

    async def update_disaster_notifications(self, db: Session) -> None:
        try:
            today_kst = get_kst_today()
            today_str = today_kst.strftime("%Y%m%d")
            
            families_with_addresses = db.query(Family.id, Account.address).join(
                Account, Family.main_user == Account.id
            ).filter(Account.address.isnot(None)).distinct().all()
        
            for family_id, address in families_with_addresses:
                family_messages = []
                
                messages_local = await self._fetch_disaster_data(address, today_str)
                if messages_local:
                    family_messages.extend(messages_local)
                
                messages_all = await self._fetch_disaster_all_data(address, today_str)
                if messages_all:
                    family_messages.extend(messages_all)
                
                unique_messages = {}
                for msg in family_messages:
                    key = msg.get('SN')
                    if key not in unique_messages:
                        unique_messages[key] = msg
                
                for message in unique_messages.values():
                    sn = message.get('SN')
                    
                    existing = db.query(Notification).filter(
                        Notification.family_id == family_id,
                        Notification.message_sn == sn
                    ).first()
                    
                    if not existing:
                        notification = Notification(
                            family_id=family_id,
                            notification_grade='WARN',
                            descriptions=json.dumps(message, ensure_ascii=False),
                            message_sn=sn
                        )
                        db.add(notification)
            
            db.commit()
            
        except Exception as e:
            logger.error(f"재난문자 업데이트 오류: {str(e)}")
            db.rollback()

    async def _fetch_disaster_data(self, address: str, today_str: str) -> List[Dict]:
        try:
            url = "https://www.safetydata.go.kr/V2/api/DSSP-IF-00247"
            
            params = {
                "serviceKey": self.api_key,
                "returnType": "json",
                "pageNo": "1",
                "numOfRows": "5",
                "rgnNm": address,
                "crtDt": today_str
            }
            
            response = requests.get(url, params=params, verify=True)
            
            if response.status_code != 200:
                logger.error(f"Disaster API error: Status {response.status_code}")
                return []
                
            data = response.json()
            
            if data['body']:
                return data["body"]
            
            logger.info(f"No disaster messages found for address: {address}")
            return []
            
        except Exception as e:
            logger.error(f"재난문자 API 호출 오류: {str(e)}")
            return []
        
    async def _fetch_disaster_all_data(self, address: str, today_str: str) -> List[Dict]:
        try:
            url = "https://www.safetydata.go.kr/V2/api/DSSP-IF-00247"
            n_add = address.split(' ')[0] + ' ' + '전체'
            params = {
                "serviceKey": self.api_key,
                "returnType": "json",
                "pageNo": "1",
                "numOfRows": "5",
                "rgnNm": n_add,
                "crtDt": today_str
            }
            
            response = requests.get(url, params=params, verify=True)
            
            if response.status_code != 200:
                logger.error(f"Disaster API error: Status {response.status_code}")
                return []
                
            data = response.json()
            
            if data['body']:
                logger.info(f'{n_add} success')
                return data["body"]
            
            logger.info(f"No disaster messages found for address: {n_add}")
            return []
            
        except Exception as e:
            logger.error(f"재난문자 API 호출 오류: {str(e)}")
            return []