"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Families Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy import and_
from sqlalchemy.exc import SQLAlchemyError

from datetime import date

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
            logger.info(f"Error getting family id from main id: {str(error)}")
            result = ""
        finally:
            return result


# 새로운 가족을 생성하는 기능
def create_family(family_data: FamiliesTable) -> bool:
    """
    새로운 가족을 생성하는 기능
    :param family_data: FamiliesTable 형식으로 미리 Mapping된 사용자 정보
    :return: 가족이 정상적으로 등록됬는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(family_data)
            logger.info(f"New family created: {family_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f" Error creating new family: {str(error)}")
        finally:
            session.commit()
            return result


# 모든 가족 정보를 불러오는 기능
def get_all_families() -> list[dict]:
    """
    모든 가족 정보를 불러오는 기능
    :return: 가족 단위로 묶은 데이터 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            family_list = session.query(
                FamiliesTable.id,
                FamiliesTable.main_user,
                FamiliesTable.family_name
            ).all()

            serialized_data: list[dict] = [{
                "id": data[0],
                "main_user": data[1],
                "family_name": data[2]
            } for data in family_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f" Error getting all family data: {str(error)}")
            result = []
        finally:
            return result

# 계정 정보로 가족을 찾는 기능
def find_family(
        user_name: str = None,
        birth_date: date = None,
        gender: Gender = None,
        address: str = None) -> list[dict]:
    """
    계정 정보를 이용해서 Family를 찾는 기능
    :param user_name: 사용자 이름
    :param birth_date: 사용자 생년월일
    :param gender: 사용자 성별
    :param address: 사용자 주소
    :return: 필터 조건에 해당하는 가족의 가족 및 계정 정보 데이터 list[dict]
    """
    result: list[dict] = []


    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            family_account_data = session.query(
                FamiliesTable.id.label('family_id'),
                FamiliesTable.family_name.label('family_name'),
                AccountsTable.id.label('account_id'),
                AccountsTable.user_name.label('account_name'),
                AccountsTable.birth_date.label('account_birth_date'),
                AccountsTable.gender.label('account_gender'),
                AccountsTable.address.label('account_address')
            ).join(
                FamiliesTable,
                FamiliesTable.main_user == AccountsTable.id
            )

            # 필터링 조건 추가
            filter_list: list  = []
            if user_name:
                filter_list.append(AccountsTable.user_name == user_name)
            if birth_date:
                filter_list.append(AccountsTable.birth_date == birth_date)
            if gender:
                filter_list.append(AccountsTable.gender == gender)
            if address:
                filter_list.append(AccountsTable.address == address)

            # 조건에 맞는 데이터 불러오기
            found_family_data = family_account_data.filter(and_(*filter_list)).all()

            serialized_data: list[dict] = [{
                "family_id": data[0],
                "family_name": data[1],
                "account_id": data[2],
                "account_name": data[3],
                "account_birth_date": data[4],
                "account_gender": data[5],
                "account_address": data[6]
            } for data in found_family_data]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f" Error getting all family data: {str(error)}")
            result = []
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
            logger.error(f" Error getting one family data: {str(error)}")
            result = {}
        finally:
            return result


# 가족 정보를 업데이트 하는 기능
def update_one_family(family_id: str, updated_family: FamiliesTable) -> bool:
    """
    가족 ID와 변경할 정보를 토대로 DB에 입력된 가족 정보를 변경하는 기능
    :param family_id: 가족의 ID
    :param updated_family: 변경할 정보가 포함된 FamiliesTable Mapping 정보
    :return: 성공적으로 변경되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            previous_family = session.query(FamiliesTable).filter(FamiliesTable.id == family_id).first()

            if previous_family is not None:
                # 가족 별명 정보가 있는 경우
                if updated_family.family_name is not None:
                    previous_family.family_name = updated_family.family_name
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f" Error updating one family data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result


# 가족 정보를 삭제하는 기능 (비밀번호 검증 필요)
def delete_one_family(family_id: str) -> bool:
    """
    가족 정보 자체를 삭제하는 기능
    :param family_id: 가족의 ID
    :return: 정상적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            family_data = session.query(FamiliesTable).filter(FamiliesTable.id == family_id).first()
            if family_data is not None:
                session.delete(family_data)
                logger.info(f"Family data deleted: {family_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f" Error deleting one family data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result