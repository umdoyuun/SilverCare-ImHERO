"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Tools Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from datetime import datetime, date

from sqlalchemy import or_, and_
from sqlalchemy.exc import SQLAlchemyError

from Utilities.logging_tools import *

logger = get_logger("DB_Tools")

# ========== Tool 부분 ==========

# 광역 자치단체 불러오는 기능
def get_all_master_region() -> list[dict]:
    """
    광역자치단체 리스트를 불러오는 기능
    :return: 전체 광역자치단체 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            region_list = session.query(
                MasterRegionsTable.region_name,
                MasterRegionsTable.region_type
            ).all()

            serialized_data: list[dict] = [{
                "region_name": data[0],
                "region_type": data[1]
            } for data in region_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all master region data: {str(error)}")
            result = []
        finally:
            return result

# 기초자치단체 불러오는 기능
def get_all_sub_region(master_region: str = None) -> list[dict]:
    """
    해당하는 master_region의 기초자치단체 리스트를 불러오는 기능
    :param master_region: 선택할 광역자치단체 이름 str (Nullable) - 입력이 없으면 모든 기초자치단체
    :return: 선택한 광역자치단체의 기초자치단체 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            region_list = None
            if master_region is not None:
                region_list = session.query(
                    SubRegionsTable.main_region,
                    SubRegionsTable.sub_region_name,
                    SubRegionsTable.region_type
                ).filter(
                    or_(
                        SubRegionsTable.main_region == master_region,  # 정확히 같은 경우
                        SubRegionsTable.main_region.like(f"%{master_region}%")  # 비슷한 경우
                    )
                ).all()
            else:
                region_list = session.query(
                    SubRegionsTable.main_region,
                    SubRegionsTable.sub_region_name,
                    SubRegionsTable.region_type
                ).all()

            serialized_data: list[dict] = [{
                "main_region": data[0],
                "sub_region_name": data[1],
                "region_type": data[2]
            } for data in region_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting all sub region data: {str(error)}")
            result = []
        finally:
            return result

# 데이터 베이스에 미리 Cache된 News를 불러오는 기능
def get_news(target_date: date) -> dict:
    """
    Cached News를 불러오는 기능
    :param target_date: News가 발행된 날짜
    :return: Category별로 정리된 News dict[list]
    """
    result: dict = {}

    category_list = [
        "business",
        "entertainment",
        "environment",
        "health",
        "politics",
        "science",
        "sports",
        "technology"
    ]

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            today_news_list = session.query(
                NewsTable.id,
                NewsTable.category,
                NewsTable.title,
                NewsTable.link,
                NewsTable.image_url,
                NewsTable.pub_date,
                NewsTable.created_at
            ).filter(NewsTable.pub_date == target_date)

            for category in category_list:
                filtered_news_list = today_news_list.filter(NewsTable.category == category).all()

                serialized_data = [{
                    "id": data[0],
                    "category": data[1],
                    "title": data[2],
                    "link": data[3],
                    "image_url": data[4],
                    "pub_date": data[5],
                    "created_at": data[6]
                } for data in filtered_news_list]

                result[category] = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting news data: {str(error)}")
            result = {}
        finally:
            return result

# Settings 값을 새롭게 추가하는 기능
def create_settings(settings_data: SettingsTable) -> bool:
    """
    Settings 값을 새롭게 추가하는 기능
    :param settings_data: SettingsTable로 미리 Mapping된 데이터
    :return: 성공적으로 생성되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(settings_data)
            logger.info(f"New settings created: {settings_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new settings: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# Settings 값을 불러오는 기능
def get_settings(family_id: str) -> dict:
    """
    해당 가족의 장비의 Settings 값을 불러오는 기능
    :param family_id: 가족의 ID
    :return: Settings 값 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            settings_data = session.query(
                SettingsTable.family_id,
                SettingsTable.is_alarm_enabled,
                SettingsTable.is_camera_enabled,
                SettingsTable.is_microphone_enabled,
                SettingsTable.is_driving_enabled
            ).filter(SettingsTable.family_id == family_id).first()

            if settings_data is not None:
                serialized_data = {
                    "family_id": settings_data[0],
                    "is_alarm_enabled": settings_data[1],
                    "is_camera_enabled": settings_data[2],
                    "is_microphone_enabled": settings_data[3],
                    "is_driving_enabled": settings_data[4]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting settings data: {str(error)}")
            result = {}
        finally:
            return result

# Settings 값을 수정하는 기능
def update_settings(family_id: str, updated_settings: SettingsTable) -> bool:
    """
    Settings 값을 업데이트하는 기능
    :param family_id: 가족의 ID
    :param updated_settings: 변경할 Settings의 값
    :return: 성공적으로 Settings 값이 업데이트 되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            previous_settings = session.query(SettingsTable).filter(SettingsTable.family_id == family_id).first()

            if previous_settings is not None:
                if updated_settings.is_alarm_enabled is not None:
                    previous_settings.is_alarm_enabled = updated_settings.is_alarm_enabled
                if updated_settings.is_camera_enabled is not None:
                    previous_settings.is_camera_enabled = updated_settings.is_camera_enabled
                if updated_settings.is_microphone_enabled is not None:
                    previous_settings.is_microphone_enabled = updated_settings.is_microphone_enabled
                if updated_settings.is_driving_enabled is not None:
                    previous_settings.is_driving_enabled = updated_settings.is_driving_enabled
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error updating settings data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# Settings 값을 삭제하는 기능
def delete_settings(family_id: str) -> bool:
    """
    Settings 값을 삭제하는 기능
    :param family_id: 가족의 ID
    :return: 성공적으로 Settings 값이 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            settings_data = session.query(SettingsTable).filter(SettingsTable.family_id == family_id).first()
            if settings_data is not None:
                session.delete(settings_data)
                logger.info(f"Settings data deleted: {settings_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting settings data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 배경화면을 추가하는 기능
def add_background(background_data: BackgroundsTable) -> bool:
    """
    배경화면을 추가하는 기능
    :param background_data: BackgroundsTable로 미리 Mapping된 데이터
    :return: 성공적으로 추가되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(background_data)
            logger.info(f"New background Added: {background_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error adding new background: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 저장된 배경화면을 불러오는 기능
def get_backgrounds(family_id: str, uploader: str = None) -> list[dict]:
    """
    저장된 배경화면을 불러오는 기능
    :param family_id: 가족의 ID
    :param uploader: 업로드 한 사용자의 ID
    :return: 조건에 맞는 배경화면 데이터 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            background_list = session.query(
                BackgroundsTable.index,
                BackgroundsTable.family_id,
                BackgroundsTable.uploader_id,
                BackgroundsTable.image_url
            )

            # 조건에 따라 필터링
            filtered_background_list = None

            if uploader is None:
                filtered_background_list = background_list.filter(BackgroundsTable.family_id == family_id).all()
            else:
                filtered_background_list = background_list.filter(
                    and_(BackgroundsTable.family_id == family_id,
                         BackgroundsTable.uploader_id == uploader)).all()

            serialized_data: list[dict] = [{
                "id": data[0],
                "family_id": data[1],
                "uploader_id": data[2],
                "image_url": data[3]
            } for data in filtered_background_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting background data: {str(error)}")
            result = []
        finally:
            return result

# 방금 업로드 된 배경화면 불러오는 기능
def get_latest_background(family_id: str, uploader:str) -> dict:
    """
    방금 업로드 된 배경하면 불러오는 기능
    :param family_id: 가족의 ID
    :param uploader: 업로드 한 사용자의 ID
    :return: 배경화면 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            background_data = session.query(
                BackgroundsTable.index,
                BackgroundsTable.family_id,
                BackgroundsTable.uploader_id,
                BackgroundsTable.image_url
            ).filter(and_(BackgroundsTable.family_id == family_id, BackgroundsTable.uploader_id == uploader)).first()

            serialized_data = {
                "index": background_data[0],
                "family_id": background_data[1],
                "uploader_id": background_data[2],
                "image_url": background_data[3]
            }

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting background data: {str(error)}")
            result = {}
        finally:
            return result

# 특정 배경화면 하나를 불러오는 기능
def get_one_background(image_id: int) -> dict:
    """
    특정 배경화면 하나를 불러오는 기능
    :param image_id: 이미지의 ID
    :return: 해당하는 배경화면 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            background_data = session.query(
                BackgroundsTable.index,
                BackgroundsTable.family_id,
                BackgroundsTable.uploader_id,
                BackgroundsTable.image_url
            ).filter(BackgroundsTable.index == image_id).first()

            serialized_data = {
                "id": background_data[0],
                "family_id": background_data[1],
                "uploader_id": background_data[2],
                "image_url": background_data[3]
            }

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting background data: {str(error)}")
            result = {}
        finally:
            return result

# 배경화면을 삭제하는 기능
def delete_background(image_id: int) -> bool:
    """
    등록된 배경화면을 삭제하는 기능
    :param image_id: 이미지의 ID
    :return: 성공적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            background_data = session.query(BackgroundsTable).filter(BackgroundsTable.index == image_id).first()
            if background_data is not None:
                session.delete(background_data)
                logger.info(f"Background data deleted: {background_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting background data: {str(error)}")
            result = False
        finally:
            session.commit()
            return result
