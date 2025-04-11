"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Members Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy import and_
from sqlalchemy.exc import SQLAlchemyError

from Utilities.logging_tools import *

logger = get_logger("DB_Members")

# 새로운 가족 관계 생성하는 기능
def create_member(member_data: MemberRelationsTable) -> bool:
    """
    새로운 가족 관계를 생성하는 기능
    :param member_data: MemberRelationsTable 형식으로 미리 Mapping된 정보
    :return: 가족 관계가 정상적으로 생성되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(member_data)
            logger.info(f"New member created: {member_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new member: {str(error)}")
            result = False
        finally:
            session.commit()
            return result


# 조건에 따른 모든 가족 관계 불러오는 기능
def get_all_members(family_id: str = None, user_id: str = None) -> list[dict]:
    """
    조건에 따른 모든 가족 관계 정보 불러오는 기능
    :param family_id: 가족 ID str (Nullable)
    :param user_id: 사용자 계정 ID str (Nullable)
    :return: 가족 관계 단위로 묶은 데이터 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            member_list: list = []

            # 주어진 조건에 따라 Query 설정
            if family_id and not user_id:
                member_list = session.query(
                    MemberRelationsTable.id,
                    MemberRelationsTable.family_id,
                    MemberRelationsTable.user_id,
                    MemberRelationsTable.nickname
                ).filter(MemberRelationsTable.family_id == family_id).all()
            elif not family_id and user_id:
                member_list = session.query(
                    MemberRelationsTable.id,
                    MemberRelationsTable.family_id,
                    MemberRelationsTable.user_id,
                    MemberRelationsTable.nickname
                ).filter(MemberRelationsTable.user_id == user_id).all()
            elif family_id and user_id:
                member_list = session.query(
                    MemberRelationsTable.id,
                    MemberRelationsTable.family_id,
                    MemberRelationsTable.user_id,
                    MemberRelationsTable.nickname
                ).filter(and_(MemberRelationsTable.family_id == family_id,
                              MemberRelationsTable.user_id == user_id)).all()
            else:
                member_list = session.query(
                    MemberRelationsTable.id,
                    MemberRelationsTable.family_id,
                    MemberRelationsTable.user_id,
                    MemberRelationsTable.nickname
                ).all()

            serialized_data: list[dict] = [{
                "id": data[0],
                "family_id": data[1],
                "user_id": data[2],
                "nickname": data[3]
            } for data in member_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all member data: {str(error)}")
            result = []
        finally:
            return result


# 가족 관계 정보를 불러오는 기능
def get_one_member(member_id: str) -> dict:
    """
    Member ID를 이용해 하나의 가족 관계 정보를 불러오는 기능
    :param member_id: 가족 관계 ID
    :return: 하나의 가족 관계 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            member_data = session.query(
                MemberRelationsTable.id,
                MemberRelationsTable.family_id,
                MemberRelationsTable.user_id,
                MemberRelationsTable.nickname
            ).filter(MemberRelationsTable.id == member_id).first()

            serialized_data: dict = {
                "id": member_data[0],
                "family_id": member_data[1],
                "user_id": member_data[2],
                "nickname": member_data[3]
            }

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting one member data: {str(error)}")
            result = {}
        finally:
            return result


# 가족 관계 정보를 업데이트 하는 기능
def update_one_member(member_id: str, updated_member: MemberRelationsTable) -> bool:
    """
    가족 관계 ID와 변경할 정보를 토대로 DB에 입력된 정보를 변경하는 기능
    :param member_id: 가족 관계를 나타내는 ID
    :param updated_member: 변경할 정보가 포함된 MemberRelationsTable Mapping 정보
    :return: 성공적으로 변경되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            previous_member = session.query(MemberRelationsTable).filter(MemberRelationsTable.id == member_id).first()

            if previous_member is not None:
                # 가족 관계 별명 정보가 있는 경우
                if updated_member.nickname is not None:
                    previous_member.nickname = updated_member.nickname
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error updating one member data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result


# 가족 관계 정보를 삭제하는 기능
def delete_one_member(member_id: str) -> bool:
    """
    가족 관계 정보 자체를 삭제하는 기능
    :param member_id: 가족 관계의 ID
    :return: 정상적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            member_data = session.query(MemberRelationsTable).filter(MemberRelationsTable.id == member_id).first()
            if member_data is not None:
                session.delete(member_data)
                logger.info(f"Member data deleted: {member_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting one member data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result