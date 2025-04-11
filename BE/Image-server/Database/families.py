"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Families Part
"""

# Library
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy.exc import SQLAlchemyError

from Utilities.logging_tools import *

logger = get_logger("DB_Families")


# 주 사용자 ID로 가족 ID를 불러오기
def main_id_to_family_id(main_id: str) -> str:
    """
    주 사용자 ID로 가족 ID를 불러오는 기능
    :param main_id: 주 사용자의 ID
    :return: 가족 ID str
    """
    result: str = ""

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            family_id = session.query(FamiliesTable.id).filter(FamiliesTable.main_user == main_id).first()

            if family_id is not None:
                result = family_id[0].__str__()
            else:
                result = ""
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting family id from main id: {str(error)}")
            result = ""
        finally:
            return result


# 가족 정보를 불러오는 기능
def get_one_family(family_id: str) -> dict:
    """
    Family ID를 이용해 하나의 가족 데이터를 불러오는 기능
    :param family_id: 가족의 ID
    :return: 하나의 가족 데이터 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            family_data = session.query(
                FamiliesTable.id,
                FamiliesTable.main_user,
                FamiliesTable.family_name
            ).filter(FamiliesTable.id == family_id).first()

            serialized_data: dict = {
                "id": family_data[0],
                "main_user": family_data[1],
                "family_name": family_data[2]
            }

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting one family data: {str(error)}")
            result = {}
        finally:
            return result
