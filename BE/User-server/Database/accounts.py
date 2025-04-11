"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Accounts Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy.exc import SQLAlchemyError

from Utilities.logging_tools import *

logger = get_logger("DB_Accounts")

# 모든 사용자의 이메일 불러오기
def get_all_email() -> list[dict]:
    """
    등록된 사용자 계정 이메일을 모두 불러오는 기능
    :return: 등록된 사용자의 모든 이메일 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            account_list = session.query(AccountsTable.email).all()
            result = [{"email": data[0]} for data in account_list]
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all email: {str(error)}")
            result = []
        finally:
            return result

# 새로운 사용자 계정 추가하기
def create_account(account_data: AccountsTable) -> bool:
    """
    새로운 사용자 계정을 만드는 기능
    :param account_data: AccountsTable 형식으로 미리 Mapping된 사용자 정보
    :return: 계정이 정상적으로 등록 됬는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(account_data)
            logger.info(f"New account created: {account_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new account: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 모든 사용자 계정 정보 불러오기
def get_all_accounts() -> list[dict]:
    """
    모든 사용자의 계정 정보를 불러오는 기능
    :return: 사용자 계정 단위로 묶은 데이터 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            account_list = session.query(
                AccountsTable.id,
                AccountsTable.email,
                AccountsTable.role,
                AccountsTable.user_name,
                AccountsTable.birth_date,
                AccountsTable.gender,
                AccountsTable.address
            ).all()

            serialized_data: list[dict] = [{
                "id": data[0],
                "email": data[1],
                "role": data[2],
                "user_name": data[3],
                "birth_date": data[4],
                "gender": data[5],
                "address": data[6]
            } for data in account_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all account data: {str(error)}")
            result = []
        finally:
            return result

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

# 사용자 이메일로부터 사용자 ID 불러오기
def get_id_from_email(email: str) -> str:
    """
    사용자 이메일 주소를 이용해 사용자 ID를 받아내는 기능
    :param email: 사용자의 이메일 주소
    :return: 찾은 사용자 ID str
    """
    user_id: str = ""

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            user_id = session.query(AccountsTable.id).filter(AccountsTable.email == email).first()[0].__str__()
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting id from email: {str(error)}")
            user_id = ""
        finally:
            return user_id

# 사용자 비밀번호 Hash 정보 불러오기
def get_hashed_password(account_id: str) -> str:
    """
    한 사용자의 Hashed 비밀번호를 불러오는 기능
    :param account_id: 사용자 ID
    :return: Hashed 비밀번호 str
    """
    result: str = ""

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            hashed_password = \
            session.query(AccountsTable.password).filter(AccountsTable.id == account_id).first()[0]
            result = hashed_password.__str__()
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting hashed password: {str(error)}")
            result = ""
        finally:
            return result

# 사용자 계정 정보 변경하기
def update_one_account(account_id: str, updated_account: AccountsTable) -> bool:
    """
    아이디와 최종으로 변경할 데이터를 이용해 계정의 정보를 변경하는 기능
    :param account_id: 사용자의 ID
    :param updated_account: 변경할 정보가 포함된 AccountsTable Mapping 정보
    :return: 정보가 성공적으로 변경되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            previous_account = session.query(AccountsTable).filter(AccountsTable.id == account_id).first()

            if previous_account is not None:
                # 이메일 정보가 있는 경우 변경
                if updated_account.email is not None:
                    previous_account.email = updated_account.email
                # 사용자 역할이 있는 경우 변경
                if updated_account.role is not None:
                    previous_account.role = updated_account.role
                # 사용자 이름이 있는 경우 변경
                if updated_account.user_name is not None:
                    previous_account.user_name = updated_account.user_name
                # 생년월일이 있는 경우 변경
                if updated_account.birth_date is not None:
                    previous_account.birth_date = updated_account.birth_date
                # 성별 정보가 있는 경우
                if updated_account.gender is not None:
                    previous_account.gender = updated_account.gender
                # 주소 정보가 있는 경우
                if updated_account.address is not None:
                    previous_account.address = updated_account.address
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error updating one account data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 사용자 계정을 삭제하는 기능 (비밀번호 검증 필요)
def delete_one_account(account_id: str) -> bool:
    """
    사용자 계정 자체를 삭제하는 기능
    :param account_id: 사용자의 ID
    :return: 삭제가 성공적으로 이뤄졌는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            account_data = session.query(AccountsTable).filter(AccountsTable.id == account_id).first()
            if account_data is not None:
                session.delete(account_data)
                logger.info(f"Account data deleted: {account_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting one account data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result
