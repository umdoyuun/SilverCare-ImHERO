"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Accounts Part
"""

# Library
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy.exc import SQLAlchemyError

from Utilities.logging_tools import *

logger = get_logger("DB_Accounts")


# 사용자 계정 정보 불러오기
def get_one_account(account_id: str) -> dict:
    """
    ID를 이용해 하나의 사용자 계정 정보를 불러오는 기능
    :param account_id: 사용자의 ID
    :return: 하나의 사용자 데이터 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            account_data = session.query(
                AccountsTable.id,
                AccountsTable.email,
                AccountsTable.role,
                AccountsTable.user_name,
                AccountsTable.birth_date,
                AccountsTable.gender,
                AccountsTable.address
            ).filter(AccountsTable.id == account_id).first()

            if account_data is not None:
                serialized_data: dict = {
                    "id": account_data[0],
                    "email": account_data[1],
                    "role": account_data[2],
                    "user_name": account_data[3],
                    "birth_date": account_data[4],
                    "gender": account_data[5],
                    "address": account_data[6]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting one account data: {str(error)}")
            result = {}
        finally:
            return result
