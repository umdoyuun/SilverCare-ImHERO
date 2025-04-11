"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Authentication Part
"""

# Library
from Database.connector import database_instance as database
from Database.models import *

from fastapi import Request

from sqlalchemy import func, and_
from sqlalchemy.exc import SQLAlchemyError

from datetime import timezone, datetime, timedelta
from time import time

from asyncio import sleep

import os
from dotenv import load_dotenv

from Utilities.logging_tools import *

logger = get_logger("DB_Authentication")

# 세션 만료 시간 불러오기
load_dotenv()
session_expire_time: int = int(os.getenv("SESSION_EXPIRE_TIME", 1800))
session_cleanup_interval: int = int(os.getenv("SESSION_CLEANUP_INTERVAL", 600))


# 현재 사용자 정보 가져오기
def check_current_user(request: Request) -> str:
    """
    요청한 자료 내의 Cookie 값을 이용해 사용자 ID를 식별하는 기능
    :param request: 사용자가 요청한 자료 덩어리
    :return: 해당 사용자의 ID str
    """
    user_id: str = ""
    session_id: str = request.cookies.get("session_id")

    # 세션 아이디가 전해준 쿠키에 포함되어 있는지 확인
    if not session_id:
        return user_id

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            login_data = session.query(LoginSessionsTable).filter(LoginSessionsTable.xid == session_id).first()

            # DB에 해당하는 세션 정보가 존재하는지 확인
            if login_data is None:
                return user_id

            # Main User가 아닌 경우 세션 만료를 적용
            if not login_data.is_main_user:
                current_time: int = int(time())
                last_active_time = login_data.last_active.replace(tzinfo=timezone.utc)
                last_active: int = int(last_active_time.timestamp())

                # 시간 초과로 세션이 만료되었는지 확인
                if current_time - last_active > session_expire_time:
                    session.delete(login_data)
                    session.commit()
                    return user_id

            # 최근 접근 기록 갱신하기
            session.query(LoginSessionsTable).filter(
                LoginSessionsTable.xid == session_id
            ).update({
                LoginSessionsTable.last_active: func.now()
            })

            user_id = login_data.user_id
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error checking current user: {str(error)}")
            user_id = ""
        finally:
            session.commit()
            return user_id
