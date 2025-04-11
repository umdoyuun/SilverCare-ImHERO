"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Authentication Part
"""


# Libraries
from Database.connector import database_instance as database
from Database.models import *

from fastapi import Request, HTTPException, status

from sqlalchemy import func, and_, or_
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
session_expire_time: int = int(os.getenv("SESSION_EXPIRE_TIME", 1800))  # 일반 사용자 만료 : 기본 - 30분
extended_session_expire_time: int = int(os.getenv("EXTENDED_SESSION_EXPIRE_TIME", 259200))  # 주 사용자 만료 : 기본 - 3일
remember_expire_time: int = int(os.getenv("REMEMBER_EXPIRE_TIME", 2592000))  # 자동 로그인 사용자 만료 : 기본 - 30일
session_cleanup_interval: int = int(os.getenv("SESSION_CLEANUP_INTERVAL", 600))  # Session 정리 주기 : 기본 - 10분

# 로그인을 위해 Session을 생성하는 기능
def create_session(session_data: LoginSessionsTable) -> bool:
    """
    로그인 처리를 위헤 Session을 생성하여 DB에 등록하는 기능
    :param session_data: Session 생성을 위해 LoginSessionsTable로 미리 Mapping된 정보
    :return: Session을 성공적으로 생성했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(session_data)
            logger.info(f"New session created: {session_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new session: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 로그아웃을 위해 세션을 삭제하는 기능
def delete_session(session_id: str) -> bool:
    """
    로그아웃 처리를 위해 Session을 DB에서 삭제하는 기능
    :param session_id: 제거할 Session의 ID
    :return: Session을 성공적으로 삭제했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session_data = session.query(LoginSessionsTable).filter(LoginSessionsTable.xid == session_id).first()
            if session_data is not None:
                session.delete(session_data)
                logger.info(f"Session deleted: {session_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting session: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

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

            # 현재 시간 및 만료 시간 확인
            current_time: int = int(time())
            last_active_time = login_data.last_active.replace(tzinfo=timezone.utc)
            last_active: int = int(last_active_time.timestamp())

            # 사용자 유형별로 세션 만료 여부 확인
            if login_data.is_main_user is True:  # Main User에 해당하는 세션 만료 확인
                if (login_data.is_remember and current_time - last_active > remember_expire_time) or \
                    (not login_data.is_remember and current_time - last_active > extended_session_expire_time):
                    session.delete(login_data)
                    session.commit()
                    return user_id
            elif login_data.is_main_user is False:  # 그외 User에 해당하는 세션 만료 확인
                if (login_data.is_remember and current_time - last_active > remember_expire_time) or \
                    (not login_data.is_remember and current_time - last_active > session_expire_time):
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

# 사용자 계정의 비밀번호를 변경하는 기능
def change_password(user_id: str, new_hashed_password: str) -> bool:
    """
    사용자 정보에 등록된 비밀번호를 변경하는 기능
    :param user_id: 비밀번호를 변경할 사용자의 ID
    :param new_hashed_password: 미리 암호화된 사용자의 새로운 비밀번호
    :return: 성공적으로 비밀번호를 변경했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            previous_account = session.query(AccountsTable).filter(AccountsTable.id == user_id).first()

            if previous_account is not None:
                # 새로운 비밀번호로 변경
                previous_account.password = new_hashed_password
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error changing password: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 세션 ID가 존재하는지 확인
def get_login_session(session_id: str) -> dict:
    """
    해당 로그인 세션이 존재하는지 확인하는 기능
    :param session_id: 세션 ID
    :return: 세션의 데이터 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            login_data = session.query(LoginSessionsTable).filter(LoginSessionsTable.xid == session_id).first()

            if login_data is not None:
                serialized_data: dict = {
                    "xid": login_data.xid,
                    "user_id": login_data.user_id,
                    "last_active": login_data.last_active,
                    "is_main_user": login_data.is_main_user,
                    "is_remember": login_data.is_remember
                }

                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting login session: {str(error)}")
            result = {}
        finally:
            return result

# 불필요한 세션 정보를 정리하는 기능
async def cleanup_login_sessions() -> None:
    while True:
        result: bool = False

        # 지정된 시간 간격으로 수행하기
        await sleep(session_cleanup_interval)

        database_pre_session = database.get_pre_session()
        with database_pre_session() as session:
            try:
                current_time: datetime = datetime.now(tz=timezone.utc)
                expired_time: datetime = current_time - timedelta(seconds=session_expire_time)
                extended_expired_time: datetime = current_time - timedelta(seconds=extended_session_expire_time)
                remember_expired_time: datetime = current_time - timedelta(seconds=remember_expire_time)

                expired_sessions = session.query(
                    LoginSessionsTable
                ).filter(or_(
                    and_(LoginSessionsTable.last_active < expired_time,
                         LoginSessionsTable.is_main_user == False,
                         LoginSessionsTable.is_remember == False),
                    and_(LoginSessionsTable.last_active < extended_expired_time,
                         LoginSessionsTable.is_main_user == True,
                         LoginSessionsTable.is_remember == False),
                    and_(LoginSessionsTable.last_active < remember_expired_time,
                         LoginSessionsTable.is_remember == True)
                )).all()

                for session_data in expired_sessions:
                    session.delete(session_data)

                result = True
            except SQLAlchemyError as error:
                session.rollback()
                logger.error(f"Error cleaning up login sessions: {str(error)}")
                result = False
            finally:
                session.commit()
                if result:
                    logger.info(f"Cleaned up login sessions")
                else:
                    logger.error(f"Failed to clean up login sessions")

# 자동 로그인을 사용하는지 기록하는 기능
def record_auto_login(session_id: str) -> bool:
    """
    Session 정보에 자동 로그인 사용을 기록하는 기능
    :param session_id: 세션 ID
    :return: 성공적으로 기록했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            login_data = session.query(LoginSessionsTable).filter(LoginSessionsTable.xid == session_id).first()
            if login_data is not None:
                login_data.is_remember = True
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error recording auto login: {str(error)}")
            result = False
        finally:
            session.commit()
            return result
