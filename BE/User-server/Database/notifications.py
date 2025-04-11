"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Notifications Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy import and_
from sqlalchemy.exc import SQLAlchemyError

from datetime import datetime, timezone

from Utilities.logging_tools import *

logger = get_logger("DB_Notifications")

# ========== Notifications 부분 ==========

# 새로운 알림을 생성하는 기능
def create_notification(notification_data: NotificationsTable) -> bool:
    """
    새로운 알림을 생성
    :param notification_data: NotificationsTable로 미리 Mapping된 Data
    "return: 알림을 성공적으로 생성했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(notification_data)
            logger.info(f"New Notification created: {notification_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new notification: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 아직 읽지 않은 알림을 가져오는 기능
def get_new_notifications(
        family_id: str,
        start_time: datetime = None,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    아직 읽지 않은 Family의 알림을 가져오는 기능
    :param family_id: 해당하는 Family의 ID str
    :param start_time: 검색을 위한 시작 날짜와 시각
    :param end_time: 검색을 위한 끝 날짜와 시각
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 읽지 않은 Family에게 도착한 모든 알림 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            new_notification_list = session.query(
                NotificationsTable.index,
                NotificationsTable.family_id,
                NotificationsTable.created_at,
                NotificationsTable.notification_grade,
                NotificationsTable.descriptions,
                NotificationsTable.is_read,
                NotificationsTable.image_url
            ).filter(and_(NotificationsTable.family_id == family_id,
                          NotificationsTable.is_read == False))

            if start_time is not None:
                filtered_new_notification_list = new_notification_list.filter(
                    NotificationsTable.created_at.between(start_time, end_time))
            else:
                filtered_new_notification_list = new_notification_list

            ordered_new_notification_list = None

            if time_order == Order.ASC:
                ordered_new_notification_list = filtered_new_notification_list.order_by(
                    NotificationsTable.created_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_new_notification_list = filtered_new_notification_list.order_by(
                    NotificationsTable.created_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "created_at": data[2],
                "notification_grade": data[3],
                "description": data[4],
                "is_read": data[5],
                "image_url": data[6]
            } for data in ordered_new_notification_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting new notifications: {str(error)}")
            result = []
        finally:
            return result

# 모든 알림을 가져오는 기능
def get_all_notifications(
        family_id: str,
        start_time: datetime = None,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    Family의 모든 알림을 가져오는 기능
    :param family_id: 해당하는 Family의 ID str
    :param start_time: 검색을 위한 시작 날짜와 시각
    :param end_time: 검색을 위한 끝 날짜와 시각
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: Family ID로 필터링 된 모든 알림 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            all_notification_list = session.query(
                NotificationsTable.index,
                NotificationsTable.family_id,
                NotificationsTable.created_at,
                NotificationsTable.notification_grade,
                NotificationsTable.descriptions,
                NotificationsTable.is_read,
                NotificationsTable.image_url
            ).filter(NotificationsTable.family_id == family_id)

            if start_time is not None:
                filtered_all_notification_list = all_notification_list.filter(
                    NotificationsTable.created_at.between(start_time, end_time))
            else:
                filtered_all_notification_list = all_notification_list

            ordered_all_notification_list = None

            if time_order == Order.ASC:
                ordered_all_notification_list = filtered_all_notification_list.order_by(
                    NotificationsTable.created_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_all_notification_list = filtered_all_notification_list.order_by(
                    NotificationsTable.created_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "created_at": data[2],
                "notification_grade": data[3],
                "description": data[4],
                "is_read": data[5],
                "image_url": data[6]
            } for data in ordered_all_notification_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all notifications: {str(error)}")
            result = []
        finally:
            return result

# Index 번호로 알림을 가져오는 기능
def get_one_notification(notification_id: int) -> dict:
    """
    Index 번호로 알림을 가져오는 기능
    :param notification_id: 알림의 Index 번호
    :return: 불러온 알림 데이터 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            notification_data = session.query(
                NotificationsTable.index,
                NotificationsTable.family_id,
                NotificationsTable.created_at,
                NotificationsTable.notification_grade,
                NotificationsTable.descriptions,
                NotificationsTable.is_read,
                NotificationsTable.image_url
            ).filter(NotificationsTable.index == notification_id).first()

            serialized_data: dict = {
                "index": notification_data[0],
                "family_id": notification_data[1],
                "created_at": notification_data[2],
                "notification_grade": notification_data[3],
                "description": notification_data[4],
                "is_read": notification_data[5],
                "image_url": notification_data[6]
            }

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting one notification: {str(error)}")
            result = {}
        finally:
            return result

# 알람을 읽었다고 기록하는 기능
def check_read_notification(notification_id: int) -> bool:
    """
    알림의 읽음을 기록하는 기능
    :param notification_id: 알림에 배정된 고유 번호
    :return: 성공적으로 기록했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            notification_data = session.query(NotificationsTable).filter(NotificationsTable.index == notification_id).first()
            if notification_data is not None:
                notification_data.is_read = True
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error checking read notifications: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 알림을 삭제하는 기능
def delete_notification(notification_id: int) -> bool:
    """
    알림을 삭제하는 기능
    :param notification_id: 알림에 배정된 고유 번호
    :return: 성공적으로 삭제했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            notification_data = session.query(NotificationsTable).filter(NotificationsTable.index == notification_id).first()
            if notification_data is not None:
                session.delete(notification_data)
                logger.info(f"Notification data deleted: {notification_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting notifications: {str(error)}")
            result = False
        finally:
            session.commit()
            return result
