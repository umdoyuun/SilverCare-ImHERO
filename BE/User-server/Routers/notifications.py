"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Parts of Notifications
"""

# Libraries
from fastapi import HTTPException, APIRouter, status, Query, Depends
from fastapi.encoders import jsonable_encoder

from datetime import datetime, timezone

import Database
from Database.models import *

from Utilities.check_tools import *
from Utilities.logging_tools import *

router = APIRouter(prefix="/notify", tags=["Notifications"])
logger = get_logger("Router_Notifications")

# ========== Notifications 부분 ==========

# 새로운 알림을 생성하는 기능
@router.post("", status_code=status.HTTP_201_CREATED)
async def crate_notification(notification_data: Notification, request_id: str = Depends(Database.check_current_user)):
    # 필수 입력 정보 점검
    missing_location: list = ["body"]

    if notification_data.family_id is None or notification_data.family_id == "":
        missing_location.append("family_id")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Family ID is required",
                "input": jsonable_encoder(notification_data)
            }
        )

    # 잘못된 옵션을 선택했는지 점검
    if notification_data.notification_grade is not None \
            and notification_data.notification_grade.lower() not in NotificationGrade._value2member_map_:
        logger.error(f"Invalid value provided for account details (notification_grade): {notification_data.notification_grade}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "type": "invalid value",
                "message": "Invalid value provided for account details (notification_grade)",
                "input": jsonable_encoder(notification_data)
            }
        )

    # 시스템 계정을 제외한 가족의 주 사용자, 보조 사용자만 알림을 생성할 수 있음
    request_data: dict = Database.get_one_account(request_id)
    family_data: dict = Database.get_one_family(notification_data.family_id)
    member_data: list = Database.get_all_members(family_id=notification_data.family_id)
    permission_id: list[str] = (([family_data["main_user"]] if family_data else []) +
                                [user_data["user_id"] for user_data in member_data])

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id not in permission_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": jsonable_encoder(notification_data)
            }
        )

    # 새로운 알림 생성
    new_notification: NotificationsTable = NotificationsTable(
        family_id=notification_data.family_id,
        notification_grade=notification_data.notification_grade.upper() \
            if notification_data.notification_grade else None,
        descriptions=notification_data.descriptions,
        image_url=notification_data.image_url
    )

    # 업로드
    result: bool = Database.create_notification(new_notification)

    if result:
        notification_data: list = Database.get_all_notifications(
            family_id=notification_data.family_id,
            time_order=Order.DESC)
        return {
            "message": "Notification created successfully",
            "result": {
                "index": notification_data[0]["index"]
            }
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to create notification",
                "input": jsonable_encoder(notification_data)
            }
        )

# 아직 읽지 않은 알림을 불러오는 기능
@router.get("/new/{family_id}", status_code=status.HTTP_200_OK)
async def get_new_notification(
        family_id: str,
        start: Optional[datetime] = Query(None, description="Query start time"),
        end: Optional[datetime] = Query(datetime.now(tz=timezone.utc), description="Query end time"),
        order: Optional[Order] = Query(Order.ASC, description="Query order"),
        request_id = Depends(Database.check_current_user)):
    # 시스템 계정을 제외한 가족의 주 사용자, 보조 사용자만 알림을 불러올 수 있음
    request_data: dict = Database.get_one_account(request_id)
    family_data: dict = Database.get_one_family(family_id)
    member_data: list = Database.get_all_members(family_id=family_id)
    permission_id: list[str] = (([family_data["main_user"]] if family_data else []) +
                                [user_data["user_id"] for user_data in member_data])

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id not in permission_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 정보 불러오기
    notification_data: list = Database.get_new_notifications(
        family_id=family_id,
        start_time=start,
        end_time=end if start else None,
        time_order=order
    )

    if notification_data:
        return {
            "message": "New notification retrieved successfully",
            "result": jsonable_encoder(notification_data)
        }
    else:
        return {
            "message": "No new notifications found",
            "result": jsonable_encoder(notification_data)
        }

# 모든 알림을 가져오는 기능
@router.get("/all/{family_id}", status_code=status.HTTP_200_OK)
async def get_all_notification(
        family_id: str,
        start: Optional[datetime] = Query(None, description="Query start time"),
        end: Optional[datetime] = Query(datetime.now(tz=timezone.utc), description="Query end time"),
        order: Optional[Order] = Query(Order.ASC, description="Query order"),
        request_id = Depends(Database.check_current_user)):
    # 시스템 계정을 제외한 가족의 주 사용자, 보조 사용자만 알림을 불러올 수 있음
    request_data: dict = Database.get_one_account(request_id)
    family_data: dict = Database.get_one_family(family_id)
    member_data: list = Database.get_all_members(family_id=family_id)
    permission_id: list[str] = (([family_data["main_user"]] if family_data else []) +
                                [user_data["user_id"] for user_data in member_data])

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id not in permission_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 정보 불러오기
    notification_data: list = Database.get_all_notifications(
        family_id=family_id,
        start_time=start,
        end_time=end if start else None,
        time_order=order
    )

    if notification_data:
        return {
            "message": "All notifications retrieved successfully",
            "result": jsonable_encoder(notification_data)
        }
    else:
        return {
            "message": "No new notifications found",
            "result": jsonable_encoder(notification_data)
        }

# 알림을 읽음 표시하는 기능
@router.patch("/read/{notification_id}", status_code=status.HTTP_200_OK)
async def read_notification(notification_id: int, request_id = Depends(Database.check_current_user)):
    # 존재하는 알림인지 확인
    notification_data: dict = Database.get_one_notification(notification_id)

    if not notification_data:
        logger.warning(f"Notification not found: {notification_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Notification not found"
            }
        )

    # 시스템 계정을 제외한 가족의 주 사용자, 보조 사용자만 알림을 불러올 수 있음
    request_data: dict = Database.get_one_account(request_id)
    family_data: dict = Database.get_one_family(notification_data["family_id"])
    member_data: list = Database.get_all_members(family_id=notification_data["family_id"])
    permission_id: list[str] = (([family_data["main_user"]] if family_data else []) +
                                [user_data["user_id"] for user_data in member_data])

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id not in permission_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 읽음 표시하기
    result: bool = Database.check_read_notification(notification_id)

    if result:
        logger.info(f"Notification check read successfully: {notification_id}")
        return {
            "message": "Notification check read successfully",
            "result": {
                **jsonable_encoder(notification_data, exclude={"is_read"}),
                "is_read": True
            }
        }
    else:
        logger.warning(f"Failed to check read notification: {notification_id}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to check read notification"
            }
        )

@router.patch("/read-many", status_code=status.HTTP_200_OK)
async def read_many_notification(index_data: IndexList, request_id = Depends(Database.check_current_user)):
    # 사용자 계정으로 요청하는지 점검
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"Can not access account: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 필수 입력 정보를 전달했는지 점검
    missing_location: list = ["body"]

    if index_data.index_list is None or len(index_data.index_list) < 1:
        missing_location.append("index_list")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Notification id data is required",
                "input": jsonable_encoder(index_data)
            }
        )

    # 읽음 처리 수행하기
    result: list[dict] = []
    is_failed: bool = False
    index_list: list[int] = index_data.index_list

    for index in index_list:
        notification_data: dict = Database.get_one_notification(index)
        request_data: dict = Database.get_one_account(request_id)
        family_data: dict = Database.get_one_family(notification_data["family_id"]) if notification_data else None
        member_data: list = Database.get_all_members(family_id=notification_data["family_id"]) if notification_data else []
        permission_id: list[str] = (([family_data["main_user"]] if family_data else []) +
                                    [user_data["user_id"] for user_data in member_data])

        if not notification_data:
            logger.warning(f"Notification not found: {index}")
            result.append({
                "index": index,
                "is_processed": False,
                "code": status.HTTP_404_NOT_FOUND,
                "message": "Notification does not exist"
            })
        elif request_data["role"] != Role.SYSTEM and request_id not in permission_id:
            logger.warning(f"You do not have permission: {request_id}")
            result.append({
                "index": index,
                "is_processed": False,
                "code": status.HTTP_403_FORBIDDEN,
                "message": "You do not have permission"
            })
        else:
            part_result: bool = Database.check_read_notification(index)

            if part_result:
                logger.info(f"Notification check read successfully: {index}")
                result.append({
                    "index": index,
                    "is_processed": True,
                    "code": status.HTTP_200_OK,
                    "message": "Notification check read successfully"
                })
            else:
                is_failed = True
                logger.error(f"Failed to check read notification: {index}")
                result.append({
                    "index": index,
                    "is_processed": False,
                    "code": status.HTTP_500_INTERNAL_SERVER_ERROR,
                    "message": "Failed to check read notification"
                })

    # 처리 결과 최종 전송
    if not is_failed:
        return {
            "message": "Results of the read notification operation",
            "result": jsonable_encoder(result)
        }
    else:
        logger.error("Failed to read many notifications")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to read many notifications",
                "result": jsonable_encoder(result)
            }
        )

# 알림을 삭제하는 기능
@router.delete("/delete/{notification_id}", status_code=status.HTTP_200_OK)
async def delete_notification(notification_id: int, request_id = Depends(Database.check_current_user)):
    # 시스템 관리자만 삭제할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or request_data["role"] != Role.SYSTEM:
        logger.warning(f"Can not access account: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 존재하는 알림인지 확인
    notification_data: dict = Database.get_one_notification(notification_id)

    if not notification_data:
        logger.warning(f"Notification not found: {notification_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Notification not found"
            }
        )

    # 삭제하기
    result: bool = Database.delete_notification(notification_id)

    if result:
        return {
            "message": "Notification deleted successfully"
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to delete notification"
            }
        )
