"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Message Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy import and_
from sqlalchemy.exc import SQLAlchemyError

from datetime import datetime, timezone

from Utilities.logging_tools import *

logger = get_logger("Database_Messages")

# ========== Messages 부분 ==========

# 새로운 메시지를 보내는 기능
def create_message(message_data: MessageTable) -> bool:
    """
    새로운 메시지를 보내는 기능
    :param message_data: MessageTable로 미리 Mapping된 Data
    :return: 성공적으로 메시지가 생성되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(message_data)
            logger.info(f"New Message created: {message_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new message: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 새롭게 수신된 메시지를 가져오는 기능
def get_new_received_messages(
        to_id: str,
        start_time: datetime = None,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    새롭게 수신된 메시지를 가져오는 기능
    :param to_id: 수신자의 ID
    :param start_time: 검색을 위한 시작 날짜와 시각
    :param end_time: 검색을 위한 끝 날짜와 시각 (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 메시지 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            received_message_list = session.query(
                MessageTable.index,
                MessageTable.from_id,
                MessageTable.to_id,
                MessageTable.created_at,
                MessageTable.content,
                MessageTable.image_url,
                MessageTable.is_read
            ).filter(and_(MessageTable.to_id == to_id, MessageTable.is_read == False))

            if start_time is not None:
                filtered_received_message_list = received_message_list.filter(
                    MessageTable.created_at.between(start_time, end_time))
            else:
                filtered_received_message_list = received_message_list

            ordered_received_message_list = None

            if time_order == Order.ASC:
                ordered_received_message_list = filtered_received_message_list.order_by(
                    MessageTable.created_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_received_message_list = filtered_received_message_list.order_by(
                    MessageTable.created_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "from_id": data[1],
                "to_id": data[2],
                "created_at": data[3],
                "content": data[4],
                "image_url": data[5],
                "is_read": data[6]
            } for data in ordered_received_message_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting new received messages: {str(error)}")
            result = []
        finally:
            return result


# 모든 수신 메시지를 가져오는 기능
def get_all_received_messages(
        to_id: str,
        start_time: datetime = None,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    모든 수신 메시지를 가져오는 기능
    :param to_id: 수신자의 ID
    :param start_time: 검색을 위한 시작 날짜와 시각
    :param end_time: 검색을 위한 끝 날짜와 시각 (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 메시지 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            received_message_list = session.query(
                MessageTable.index,
                MessageTable.from_id,
                MessageTable.to_id,
                MessageTable.created_at,
                MessageTable.content,
                MessageTable.image_url,
                MessageTable.is_read
            ).filter(MessageTable.to_id == to_id)

            if start_time is not None:
                filtered_received_message_list = received_message_list.filter(
                    MessageTable.created_at.between(start_time, end_time))
            else:
                filtered_received_message_list = received_message_list

            ordered_received_message_list = None

            if time_order == Order.ASC:
                ordered_received_message_list = filtered_received_message_list.order_by(
                    MessageTable.created_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_received_message_list = filtered_received_message_list.order_by(
                    MessageTable.created_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "from_id": data[1],
                "to_id": data[2],
                "created_at": data[3],
                "content": data[4],
                "image_url": data[5],
                "is_read": data[6]
            } for data in ordered_received_message_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all received messages: {str(error)}")
            result = []
        finally:
            return result

# 모든 송신 메시지를 가져오는 기능
def get_all_sent_messages(
        from_id: str,
        start_time: datetime = None,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    모든 송신 메시지를 가져오는 기능
    :param from_id: 송신자의 ID
    :param start_time: 검색을 위한 시작 날짜와 시각
    :param end_time: 검색을 위한 끝 날짜와 시각 (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 메시지 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            sent_message_list = session.query(
                MessageTable.index,
                MessageTable.from_id,
                MessageTable.to_id,
                MessageTable.created_at,
                MessageTable.content,
                MessageTable.image_url,
                MessageTable.is_read
            ).filter(MessageTable.from_id == from_id)

            if start_time is not None:
                filtered_sent_message_list = sent_message_list.filter(
                    MessageTable.created_at.between(start_time, end_time))
            else:
                filtered_sent_message_list = sent_message_list

            ordered_sent_message_list = None

            if time_order == Order.ASC:
                ordered_sent_message_list = filtered_sent_message_list.order_by(
                    MessageTable.created_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_sent_message_list = filtered_sent_message_list.order_by(
                    MessageTable.created_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "from_id": data[1],
                "to_id": data[2],
                "created_at": data[3],
                "content": data[4],
                "image_url": data[5],
                "is_read": data[6]
            } for data in ordered_sent_message_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all sent messages: {str(error)}")
            result = []
        finally:
            return result

# 특정 메시지를 가져오는 기능
def get_one_message(message_id: int) -> dict:
    """
    메시지 번호를 이용해 특정 메시지를 가져오는 기능
    :param message_id: 메시지의 고유 index
    :return: 메시지 데이터 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            message_data = session.query(
                MessageTable.index,
                MessageTable.from_id,
                MessageTable.to_id,
                MessageTable.created_at,
                MessageTable.content,
                MessageTable.image_url,
                MessageTable.is_read
            ).filter(MessageTable.index == message_id).first()

            serialized_data: dict = {
                "index": message_data[0],
                "from_id": message_data[1],
                "to_id": message_data[2],
                "created_at": message_data[3],
                "content": message_data[4],
                "image_url": message_data[5],
                "is_read": message_data[6]
            }

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting one message: {str(error)}")
            result = {}
        finally:
            return result

# 메시지를 읽었음을 기록하는 기능
def check_read_message(message_id: int) -> bool:
    """
    메시지를 읽었음을 기록하는 기능
    :param message_id: 메시지의 고유 index
    :return: 성공적으로 메시지 읽음을 표시했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            message_data = session.query(MessageTable).filter(MessageTable.index == message_id).first()
            if message_data is not None:
                message_data.is_read = True
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error checking read message: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 메시지를 삭제하는 기능
def delete_message(message_id: int) -> bool:
    """
    특정 메시지를 삭제하는 기능
    :param message_id: 메시지의 고유 index
    :return: 성공적으로 메시지를 삭제했는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            message_data = session.query(MessageTable).filter(MessageTable.index == message_id).first()
            if message_data is not None:
                session.delete(message_data)
                logger.info(f"Message data deleted: {message_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting message: {str(error)}")
            result = False
        finally:
            session.commit()
            return result
