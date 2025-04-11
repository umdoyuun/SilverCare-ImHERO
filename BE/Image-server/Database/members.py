"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Members Part
"""

# Library
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy import and_
from sqlalchemy.exc import SQLAlchemyError

from Utilities.logging_tools import *

logger = get_logger("DB_Members")


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
