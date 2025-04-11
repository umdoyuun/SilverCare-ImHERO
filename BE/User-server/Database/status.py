"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Status Part
"""

# Libraries
from Database.connector import database_instance as database
from Database.models import *

from sqlalchemy import or_, and_
from sqlalchemy.exc import SQLAlchemyError

from datetime import datetime, timezone

from Utilities.logging_tools import *

logger = get_logger("DB_Status")

# ========== Home 부분 ==========

# 새로운 집 환경 정보 추가하기
def create_home_status(home_status_data: HomeStatusTable) -> bool:
    """
    새로운 집 환경 정보를 추가하는 기능
    :param home_status_data: HomeStatusTable로 미리 Mapping된 정보
    :return: 집 환경 정보가 성공적으로 추가되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(home_status_data)
            logger.info(f"New home status created: {home_status_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new home status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 조건에 따른 모든 집 환경 정보 불러오기
def get_home_status(
        family_id: str,
        start_time: datetime,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    조건에 따른 모든 집 환경 정보를 불러오는 기능
    :param family_id: 대상의 가족 ID
    :param start_time: 검색 범위의 시작 값 datetime
    :param end_time: 검색 범위의 끝 값 datetime (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 모든 집 환경 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            home_status_list = session.query(
                    HomeStatusTable.index,
                    HomeStatusTable.family_id,
                    HomeStatusTable.reported_at,
                    HomeStatusTable.temperature,
                    HomeStatusTable.humidity,
                    HomeStatusTable.dust_level,
                    HomeStatusTable.ethanol,
                    HomeStatusTable.others
                ).filter(and_(HomeStatusTable.reported_at.between(start_time, end_time),
                              HomeStatusTable.family_id == family_id))

            ordered_home_status_list = None

            if time_order == Order.ASC:
                ordered_home_status_list = home_status_list.order_by(
                    HomeStatusTable.reported_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_home_status_list = home_status_list.order_by(
                    HomeStatusTable.reported_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "reported_at": data[2],
                "temperature": data[3],
                "humidity": data[4],
                "dust_level": data[5],
                "ethanol": data[6],
                "others": data[7]
            } for data in ordered_home_status_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting home status: {str(error)}")
            result = []
        finally:
            return result

# 가장 최신의 집 환경 정보 불러오기
def get_latest_home_status(family_id: str) -> dict:
    """
    가장 최신의 집 환경 정보를 불러오기
    :param family_id: 대상의 가족 ID
    :return: 최신의 집 환경 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            home_status_data = session.query(
                HomeStatusTable.index,
                HomeStatusTable.family_id,
                HomeStatusTable.reported_at,
                HomeStatusTable.temperature,
                HomeStatusTable.humidity,
                HomeStatusTable.dust_level,
                HomeStatusTable.ethanol,
                HomeStatusTable.others
            ).filter(HomeStatusTable.family_id == family_id
            ).order_by(HomeStatusTable.reported_at.desc()).first()

            if home_status_data is not None:
                serialized_data: dict = {
                    "index": home_status_data[0],
                    "family_id": home_status_data[1],
                    "reported_at": home_status_data[2],
                    "temperature": home_status_data[3],
                    "humidity": home_status_data[4],
                    "dust_level": home_status_data[5],
                    "ethanol": home_status_data[6],
                    "others": home_status_data[7]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting latest home status: {str(error)}")
            result = {}
        finally:
            return result

# 가장 최신의 집 환경 정보 삭제하기
def delete_latest_home_status(family_id: str) -> bool:
    """
    가장 최신의 집 환경 정보를 삭제하는 기능
    :param family_id: 대상의 가족 ID
    :return: 집 환경 정보가 성공적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            home_status_data = session.query(
                HomeStatusTable
            ).filter(HealthStatusTable.family_id == family_id
            ).order_by(HomeStatusTable.reported_at.desc()).first()

            if home_status_data is not None:
                session.delete(home_status_data)
                logger.info(f"Latest home status deleted: {home_status_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting latest home status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# ========== Health 부분 ==========

# 새로운 건강 정보 추가하기
def create_health_status(health_status_data: HealthStatusTable) -> bool:
    """
    새로운 건강 정보를 추가하는 기능
    :param health_status_data: HealthStatusTable로 미리 Mapping된 정보
    :return: 건강 정보가 성공적으로 추가되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(health_status_data)
            logger.info(f"New health status created: {health_status_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new health status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 조건에 따른 모든 건강 정보 불러오기
def get_health_status(
        family_id: str,
        start_time: datetime,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    조건에 따른 모든 건강 정보를 불러오는 기능
    :param family_id: 대상의 가족 ID
    :param start_time: 검색 범위의 시작 값 datetime
    :param end_time: 검색 범위의 끝 값 datetime (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 모든 건강 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            health_status_list = health_status_list = session.query(
                    HealthStatusTable.index,
                    HealthStatusTable.family_id,
                    HealthStatusTable.reported_at,
                    HealthStatusTable.heart_rate
                ).filter(and_(HealthStatusTable.reported_at.between(start_time, end_time),
                              HealthStatusTable.family_id == family_id))

            ordered_health_status_list = None

            if time_order == Order.ASC:
                ordered_health_status_list = health_status_list.order_by(
                    HealthStatusTable.reported_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_health_status_list = health_status_list.order_by(
                    HealthStatusTable.reported_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "reported_at": data[2],
                "heart_rate": data[3]
            } for data in ordered_health_status_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting health status: {str(error)}")
            result = []
        finally:
            return result

# 가장 최신의 건강 정보 불러오기
def get_latest_health_status(family_id: str) -> dict:
    """
    가장 최신의 건강 정보를 불러오기
    :param family_id: 대상의 가족 ID
    :return: 최신의 건강 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            health_status_data = session.query(
                HealthStatusTable.index,
                HealthStatusTable.family_id,
                HealthStatusTable.reported_at,
                HealthStatusTable.heart_rate
            ).filter(HealthStatusTable.family_id == family_id
            ).order_by(HealthStatusTable.reported_at.desc()).first()

            if health_status_data is not None:
                serialized_data: dict = {
                    "index": health_status_data[0],
                    "family_id": health_status_data[1],
                    "reported_at": health_status_data[2],
                    "heart_rate": health_status_data[3]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting latest health status: {str(error)}")
            result = {}
        finally:
            return result

# 가장 최신의 건강 정보 삭제하기
def delete_latest_health_status(family_id: str) -> bool:
    """
    가장 최신의 건강 정보를 삭제하는 기능
    :param family_id: 대상의 가족 ID
    :return: 건강 정보가 성공적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            health_status_data = session.query(
                HealthStatusTable
            ).filter(HealthStatusTable.family_id == family_id
            ).order_by(HealthStatusTable.reported_at.desc()).first()

            if health_status_data is not None:
                session.delete(health_status_data)
                logger.info(f"Latest health status deleted: {health_status_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting latest health status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# ========== Active 부분 ==========

# >>> Deprecated <<<
# 새로운 활동 정보 추가하기
def create_active_status(active_status_data: ActiveStatusTable) -> bool:
    """
    새로운 활동 정보를 추가하는 기능
    :param active_status_data: ActiveStatusTable로 미리 Mapping된 정보
    :return:활동 정보가 성공적으로 추가되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            session.add(active_status_data)
            logger.info(f"New active status created: {active_status_data}")
            result = True
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error creating new active status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# >>> Deprecated <<<
# 조건에 따른 모든 활동 정보 불러오기
def get_active_status(
        family_id: str,
        start_time: datetime,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    조건에 따른 모든 활동 정보를 불러오는 기능
    :param family_id: 대상의 가족 ID
    :param start_time: 검색 범위의 시작 값 datetime
    :param end_time: 검색 범위의 끝 값 datetime (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 모든 활동 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            active_status_list = session.query(
                    ActiveStatusTable.index,
                    ActiveStatusTable.family_id,
                    ActiveStatusTable.reported_at,
                    ActiveStatusTable.score,
                    ActiveStatusTable.action,
                    ActiveStatusTable.is_critical,
                    ActiveStatusTable.description,
                    ActiveStatusTable.image_url
                ).filter(and_(ActiveStatusTable.reported_at.between(start_time, end_time),
                              ActiveStatusTable.family_id == family_id))

            ordered_active_status_list = None

            if time_order == Order.ASC:
                ordered_active_status_list = active_status_list.order_by(
                    ActiveStatusTable.reported_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_active_status_list = active_status_list.order_by(
                    ActiveStatusTable.reported_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "reported_at": data[2],
                "score": data[3],
                "action": data[4],
                "is_critical": data[5],
                "description": data[6],
                "image_url": data[7]
            } for data in ordered_active_status_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting active status: {str(error)}")
            result = []
        finally:
            return result

# >>> Deprecated <<<
# 가장 최신의 활동 정보 불러오기
def get_latest_active_status(family_id: str) -> dict:
    """
    가장 최신의 활동 정보를 불러오기
    :param family_id: 대상의 가족 ID
    :return: 최신의 활동 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            active_status_data = session.query(
                ActiveStatusTable.index,
                ActiveStatusTable.family_id,
                ActiveStatusTable.reported_at,
                ActiveStatusTable.score,
                ActiveStatusTable.action,
                ActiveStatusTable.is_critical,
                ActiveStatusTable.description,
                ActiveStatusTable.image_url
            ).filter(ActiveStatusTable.family_id == family_id
            ).order_by(ActiveStatusTable.reported_at.desc()).first()

            if active_status_data is not None:
                serialized_data: dict = {
                    "index": active_status_data[0],
                    "family_id": active_status_data[1],
                    "reported_at": active_status_data[2],
                    "score": active_status_data[3],
                    "action": active_status_data[4],
                    "is_critical": active_status_data[5],
                    "description": active_status_data[6],
                    "image_url": active_status_data[7]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting latest active status: {str(error)}")
            result = {}
        finally:
            return result

# >>> Deprecated <<<
# 가장 최신의 활동 정보 삭제하기
def delete_latest_active_status(family_id: str) -> bool:
    """
    가장 최신의 활동 정보를 삭제하는 기능
    :param family_id: 대상의 가족 ID
    :return: 활동 정보가 성공적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            active_status_data = session.query(
                ActiveStatusTable
            ).filter(ActiveStatusTable.family_id == family_id
            ).order_by(ActiveStatusTable.reported_at.desc()).first()

            if active_status_data is not None:
                session.delete(active_status_data)
                logger.info(f"Latest active status deleted: {active_status_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting latest active status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# ========== Mental 부분 ==========

# 조건에 따른 모든 정신건강 정보 불러오기
def get_mental_status(
        family_id: str,
        start_time: datetime,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list[dict]:
    """
    조건에 따른 모든 정신건강 정보를 불러오는 기능
    :param family_id: 대상의 가족 ID
    :param start_time: 검색 범위의 시작 값 datetime
    :param end_time: 검색 범위의 끝 값 datetime (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 모든 정신건강 정보 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            mental_status_list = session.query(
                    MentalStatusTable.index,
                    MentalStatusTable.family_id,
                    MentalStatusTable.reported_at,
                    MentalStatusTable.score,
                    MentalStatusTable.is_critical,
                    MentalStatusTable.description
                ).filter(and_(MentalStatusTable.reported_at.between(start_time, end_time),
                              MentalStatusTable.family_id == family_id))

            ordered_mental_status_list = None

            if time_order == Order.ASC:
                ordered_mental_status_list = mental_status_list.order_by(
                    MentalStatusTable.reported_at.asc()).all()
            elif time_order == Order.DESC:
                ordered_mental_status_list = mental_status_list.order_by(
                    MentalStatusTable.reported_at.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "reported_at": data[2],
                "score": data[3],
                "is_critical": data[4],
                "description": data[5]
            } for data in ordered_mental_status_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting mental status: {str(error)}")
            result = []
        finally:
            return result

# 가장 최신의 정신건강 정보 불러오기
def get_latest_mental_status(family_id: str) -> dict:
    """
    가장 최신의 정신건강 정보를 불러오기
    :param family_id: 대상의 가족 ID
    :return: 최신의 정신건강 정보 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            mental_status_data = session.query(
                MentalStatusTable.index,
                MentalStatusTable.family_id,
                MentalStatusTable.reported_at,
                MentalStatusTable.score,
                MentalStatusTable.is_critical,
                MentalStatusTable.description
            ).filter(MentalStatusTable.family_id == family_id
            ).order_by(MentalStatusTable.reported_at.desc()).first()

            if mental_status_data is not None:
                serialized_data: dict = {
                    "index": mental_status_data[0],
                    "family_id": mental_status_data[1],
                    "reported_at": mental_status_data[2],
                    "score": mental_status_data[3],
                    "is_critical": mental_status_data[4],
                    "description": mental_status_data[5]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting latest mental status: {str(error)}")
            result = {}
        finally:
            return result

# 가장 최신의 정신건강 정보 삭제하기
def delete_latest_mental_status(family_id: str) -> bool:
    """
    가장 최신의 정신건강 정보를 삭제하는 기능
    :param family_id: 대상의 가족 ID
    :return: 정신건강 정보가 성공적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            mental_status_data = session.query(
                MentalStatusTable
            ).filter(MentalStatusTable.family_id == family_id
            ).order_by(MentalStatusTable.reported_at.desc()).first()

            if mental_status_data is not None:
                session.delete(mental_status_data)
                logger.info(f"Latest mental status deleted: {mental_status_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting latest mental status: {str(error)}")
            result = False
        finally:
            session.commit()
            return result

# 조건에 따른 모든 정신건강 리포트 불러오기
def get_mental_reports(
        family_id: str,
        start_time: datetime,
        end_time: datetime = datetime.now(tz=timezone.utc),
        time_order: Order = Order.ASC) -> list:
    """
    조건에 따른 모든 정신건강 리포트를 불러오는 기능
    :param family_id: 대상의 가족 ID
    :param start_time: 검색 범위의 시작 값 datetime
    :param end_time: 검색 범위의 끝 값 datetime (기본은 현재)
    :param time_order: 데이터의 정렬 순서 (시간 기반)
    :return: 조건에 맞는 모든 정신건강 리포트 list[dict]
    """
    result: list[dict] = []

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            mental_reports_list = session.query(
                    MentalReportsTable.index,
                    MentalReportsTable.family_id,
                    MentalReportsTable.reported_at,
                    MentalReportsTable.start_time,
                    MentalReportsTable.end_time,
                    MentalReportsTable.average_score,
                    MentalReportsTable.critical_days,
                    MentalReportsTable.best_day,
                    MentalReportsTable.worst_day,
                    MentalReportsTable.improvement_needed,
                    MentalReportsTable.summary
                ).filter(and_(or_(MentalReportsTable.start_time.between(start_time, end_time),
                             MentalReportsTable.end_time.between(start_time, end_time)),
                              MentalReportsTable.family_id == family_id))

            ordered_mental_reports_list = None

            if time_order == Order.ASC:
                ordered_mental_reports_list = mental_reports_list.order_by(
                    MentalReportsTable.start_time.asc()).all()
            elif time_order == Order.DESC:
                ordered_mental_reports_list = mental_reports_list.order_by(
                    MentalReportsTable.start_time.desc()).all()

            serialized_data: list[dict] = [{
                "index": data[0],
                "family_id": data[1],
                "reported_at": data[2],
                "start_time": data[3],
                "end_time": data[4],
                "average_score": data[5],
                "critical_days": data[6],
                "best_day": data[7],
                "worst_day": data[8],
                "improvement_needed": data[9],
                "summary": data[10]
            } for data in ordered_mental_reports_list]

            result = serialized_data
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting mental reports: {str(error)}")
            result = []
        finally:
            return result

# 가장 최신의 정신건강 리포트 불러오기
def get_latest_mental_reports(family_id: str) -> dict:
    """
    가장 최신의 정신건강 리포트를 불러오기
    :param family_id: 대상의 가족 ID
    :return: 최신의 정신건강 리포트 dict
    """
    result: dict = {}

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            mental_reports_data = session.query(
                MentalReportsTable.index,
                MentalReportsTable.family_id,
                MentalReportsTable.reported_at,
                MentalReportsTable.start_time,
                MentalReportsTable.end_time,
                MentalReportsTable.average_score,
                MentalReportsTable.critical_days,
                MentalReportsTable.best_day,
                MentalReportsTable.worst_day,
                MentalReportsTable.improvement_needed,
                MentalReportsTable.summary
            ).filter(MentalReportsTable.family_id == family_id
            ).order_by(MentalReportsTable.reported_at.desc()).first()

            if mental_reports_data is not None:
                serialized_data: dict = {
                    "index": mental_reports_data[0],
                    "family_id": mental_reports_data[1],
                    "reported_at": mental_reports_data[2],
                    "start_time": mental_reports_data[3],
                    "end_time": mental_reports_data[4],
                    "average_score": mental_reports_data[5],
                    "critical_days": mental_reports_data[6],
                    "best_day": mental_reports_data[7],
                    "worst_day": mental_reports_data[8],
                    "improvement_needed": mental_reports_data[9],
                    "summary": mental_reports_data[10]
                }
                result = serialized_data
            else:
                result = {}
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error getting latest mental reports: {str(error)}")
            result = {}
        finally:
            return result

# 가장 최신의 정신건강 리포트 삭제하기
def delete_latest_mental_reports(family_id: str) -> bool:
    """
    가장 최신의 정신건강 리포트를 삭제하는 기능
    :param family_id: 대상의 가족 ID
    :return: 정신건강 리포트가 성공적으로 삭제되었는지 여부 bool
    """
    result: bool = False

    database_pre_session = database.get_pre_session()
    with database_pre_session() as session:
        try:
            mental_reports_data = session.query(
                MentalReportsTable
            ).filter(MentalReportsTable.family_id == family_id
            ).order_by(MentalReportsTable.reported_at.desc()).first()

            if mental_reports_data is not None:
                session.delete(mental_reports_data)
                logger.info(f"Latest mental reports deleted: {mental_reports_data}")
                result = True
            else:
                result = False
        except SQLAlchemyError as error:
            session.rollback()
            logger.error(f"Error deleting latest mental reports: {str(error)}")
            result = False
        finally:
            session.commit()
            return result
