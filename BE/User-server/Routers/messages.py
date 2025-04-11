"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Parts of Message
"""

# Libraries
from fastapi import HTTPException, APIRouter, status, Query, Depends
from fastapi.encoders import jsonable_encoder

from datetime import datetime, timezone

import Database
from Database.models import *

from Utilities.check_tools import *
from Utilities.logging_tools import *

router = APIRouter(prefix="/messages", tags=["Messages"])
logger = get_logger("Router_Messages")

# ========== Message 부분 ==========

# 메시지를 보낼 수 있는 대상을 확인하는 기능
@router.get("/receivable/{user_id}", status_code=status.HTTP_200_OK)
async def get_receivable_account(user_id: str, request_id: str = Depends(Database.check_current_user)):
    # 시스템 계정을 제외한 본인 계정에 대해서만 조회 가능
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id != user_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 시스템 계정은 모든 사람에게 보낼 수 있음
    target_account: dict = Database.get_one_account(user_id)

    if target_account["role"] == Role.SYSTEM:
        raise HTTPException(
            status_code=status.HTTP_204_NO_CONTENT,
            detail={
                "type": "no content",
                "message": "You can send messages to all users"
            }
        )

    # 접근 권한 범위 확인
    receivable_account: list[dict] = []

    if target_account["role"] == Role.MAIN:  # 주 사용자가 접근한 경우 소속된 가족까지 접근 가능
        family_id: str = Database.main_id_to_family_id(user_id)
        member_data: list[dict] = Database.get_all_members(family_id=family_id)
        for member in member_data:
            receivable_account.append({
                "user_id": member["user_id"],
                "name": member["nickname"]
            })
    elif target_account["role"] == Role.SUB:  # 보조 사용자가 접근한 경우 소속된 주 사용자들의 정보까지 접근 가능
        member_data: list[dict] = Database.get_all_members(user_id=user_id)
        family_id_list: list[str] = [member["family_id"] for member in member_data]
        for family_id in family_id_list:
            family_data: dict = Database.get_one_family(family_id)
            account_data: dict = Database.get_one_account(family_data["main_user"])
            receivable_account.append({
                "user_id": account_data["id"],
                "name": account_data["user_name"]
            })

    if receivable_account:
        return {
            "message": "Receivable account retrieved successfully",
            "result": jsonable_encoder(receivable_account)
        }
    else:
        logger.warning("No receivable account")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "No receivable account"
            }
        )


# 메시지를 보내는 기능
@router.post("/send", status_code=status.HTTP_201_CREATED)
async def send_message(message_data: Message, request_id: str = Depends(Database.check_current_user)):
    # 필수 입력 정보를 전달했는지 점검
    missing_location: list = ["body"]

    if message_data.from_id is None or message_data.from_id == "":
        missing_location.append("from_id")
    if message_data.to_id is None or message_data.to_id == "":
        missing_location.append("to_id")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Account id data is required",
                "input": jsonable_encoder(message_data)
            }
        )

    # 시스템 계정을 제외하고 보내는 사람은 요청한 사람과 같아야 함
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id != message_data.from_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": jsonable_encoder(message_data)
            }
        )

    # 수신자가 존재하는지 점검
    received_account: dict = Database.get_one_account(message_data.to_id)

    if not received_account:
        logger.warning(f"User does not exist: {message_data.to_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Received user does not exist",
                "input": jsonable_encoder(message_data)
            }
        )

    # 접근 권한 범위 설정
    accessible_id: list[str] = [request_id]

    if request_data["role"] == Role.MAIN:  # 주 사용자가 접근한 경우 소속된 가족까지 접근 가능
        family_id: str = Database.main_id_to_family_id(request_id)
        member_data: list[dict] = Database.get_all_members(family_id=family_id)
        for member in member_data:
            accessible_id.append(member["user_id"])
    elif request_data["role"] == Role.SUB:  # 보조 사용자가 접근한 경우 소속된 주 사용자들의 이미지까지 접근 가능
        member_data: list[dict] = Database.get_all_members(user_id=request_id)
        family_id_list: list[str] = [member["family_id"] for member in member_data]
        for family_id in family_id_list:
            family_data: dict = Database.get_one_family(family_id)
            accessible_id.append(family_data["main_user"])

    # 시스템 계정을 제외하고 받는 사람은 지정된 사용자 내에 있어야 함
    if request_data["role"] != Role.SYSTEM and message_data.to_id not in accessible_id:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": jsonable_encoder(message_data)
            }
        )

    # 새로운 메시지 생성
    new_message: MessageTable = MessageTable(
        from_id=message_data.from_id,
        to_id=message_data.to_id,
        content=message_data.content,
        image_url=message_data.image_url
    )

    # 업로드
    result: bool = Database.create_message(new_message)

    if result:
        logger.info(f"Message sent successfully: {message_data.from_id} -> {message_data.to_id}")
        return {
            "message": "Message sent successfully"
        }
    else:
        logger.error(f"Failed to send message: {message_data.from_id} -> {message_data.to_id}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to send message",
                "input": jsonable_encoder(message_data)
            }
        )

# 새로운 메시지를 불러오는 기능
@router.get("/new", status_code=status.HTTP_200_OK)
async def get_new_received_messages(
        start: Optional[datetime] = Query(None, description="Query start time"),
        end: Optional[datetime] = Query(datetime.now(tz=timezone.utc), description="Query end time"),
        order: Optional[Order] = Query(Order.ASC, description="Query order"),
        request_id: str = Depends(Database.check_current_user)):
    # 사용자 계정으로 접근하는지 점검
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 메시지 불러오기
    message_data: list = Database.get_new_received_messages(
        to_id=request_id,
        start_time=start,
        end_time=end if start else None,
        time_order=order
    )

    if message_data:
        return {
            "message": "New received messages retrieved successfully",
            "result": jsonable_encoder(message_data)
        }
    else:
        return {
            "message": "No new received messages",
            "result": jsonable_encoder(message_data)
        }

# 받은 모든 메시지를 불러오는 기능
@router.get("/all", status_code=status.HTTP_200_OK)
async def get_all_received_messages(
        start: Optional[datetime] = Query(None, description="Query start time"),
        end: Optional[datetime] = Query(datetime.now(tz=timezone.utc), description="Query end time"),
        order: Optional[Order] = Query(Order.ASC, description="Query order"),
        request_id: str = Depends(Database.check_current_user)):
    # 사용자 계정으로 접근하는지 점검
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 메시지 불러오기
    message_data: list = Database.get_all_received_messages(
        to_id=request_id,
        start_time=start,
        end_time=end if start else None,
        time_order=order
    )

    if message_data:
        return {
            "message": "All received messages retrieved successfully",
            "result": jsonable_encoder(message_data)
        }
    else:
        return {
            "message": "No received messages",
            "result": jsonable_encoder(message_data)
        }

# 보낸 메시지를 불러오는 기능
@router.get("/sent", status_code=status.HTTP_200_OK)
async def get_all_sent_messages(
        start: Optional[datetime] = Query(None, description="Query start time"),
        end: Optional[datetime] = Query(datetime.now(tz=timezone.utc), description="Query end time"),
        order: Optional[Order] = Query(Order.ASC, description="Query order"),
        request_id: str = Depends(Database.check_current_user)):
    # 사용자 계정으로 접근하는지 점검
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 메시지 불러오기
    message_data: list = Database.get_all_sent_messages(
        from_id=request_id,
        start_time=start,
        end_time=end if start else None,
        time_order=order
    )

    if message_data:
        return {
            "message": "All sent messages retrieved successfully",
            "result": jsonable_encoder(message_data)
        }
    else:
        logger.warning("No sent messages")
        return {
            "message": "No sent messages",
            "result": jsonable_encoder(message_data)
        }

# 메시지를 읽음 처리하는 기능
@router.patch("/read/{message_id}", status_code=status.HTTP_200_OK)
async def read_message(message_id: int, request_id: str = Depends(Database.check_current_user)):
    # 존재하는 메시지인지 확인
    message_data: dict = Database.get_one_message(message_id)

    if not message_data:
        logger.warning(f"Message does not exist: {message_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Message does not exist"
            }
        )

    # 수신자만이 메시지의 읽음 처리를 수행할 수 있음
    request_data = Database.get_one_account(request_id)

    if not request_data or message_data["to_id"] != request_id:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 읽음 표시하기
    result: bool = Database.check_read_message(message_id)

    if result:
        logger.info(f"Message read successfully: {message_id}")
        return {
            "message": "Message read successfully",
            "result": {
                **jsonable_encoder(message_data, exclude={"is_read"}),
                "is_read": True
            }
        }
    else:
        logger.error(f"Failed to read message: {message_id}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to check read message"
            }
        )

@router.patch("/read-many", status_code=status.HTTP_200_OK)
async def read_many_message(index_data: IndexList, request_id: str = Depends(Database.check_current_user)):
    # 사용자 계정으로 요청하는지 점검
    request_data = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"You do not have permission: {request_id}")
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
                "message": "Message id data is required",
                "input": jsonable_encoder(index_data)
            }
        )
    
    # 읽음 처리 수행하기
    result: list[dict] = []
    is_failed: bool = False
    index_list: list[int] = index_data.index_list

    for index in index_list:
        # 메시지가 존재하는지, 수신자와 요청자가 같은지 확인
        message_data: dict = Database.get_one_message(index)
        if not message_data:
            logger.warning(f"Message does not exist: {index}")
            result.append({
                "index": index,
                "is_processed": False,
                "code": status.HTTP_404_NOT_FOUND,
                "message": "This message does not exist"
            })
        elif message_data["to_id"] != request_id:
            logger.warning(f"You do not have permission: {request_id}")
            result.append({
                "index": index,
                "is_processed": False,
                "code": status.HTTP_403_FORBIDDEN,
                "message": "You do not have permission"
            })
        else:  # 문제가 없는 경우 수행
            part_result: bool = Database.check_read_message(index)

            if part_result:
                logger.info(f"Message read successfully: {index}")
                result.append({
                    "index": index,
                    "is_processed": True,
                    "code": status.HTTP_200_OK,
                    "message": "Message read successfully"
                })
            else:
                is_failed = True
                logger.error(f"Failed to read message: {index}")
                result.append({
                    "index": index,
                    "is_processed": False,
                    "code": status.HTTP_500_INTERNAL_SERVER_ERROR,
                    "message": "Failed to check read message"
                })


    # 처리 결과 최종 전송
    if not is_failed:
        return {
            "message": "Results of the read message operation",
            "result": jsonable_encoder(result)
        }
    else:
        logger.error("Failed to read many messages")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to read messages",
                "result": jsonable_encoder(result)
            }
        )
        



# 메시지를 삭제하는 기능
@router.delete("/delete/{message_id}", status_code=status.HTTP_200_OK)
async def delete_message(message_id: int, request_id: str = Depends(Database.check_current_user)):
    # 시스템 관리자만 삭제할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or request_data["role"] != Role.SYSTEM:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 존재하는 메시지인지 확인
    message_data: dict = Database.get_one_message(message_id)

    if not message_data:
        logger.warning(f"Message does not exist: {message_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Message does not exist"
            }
        )

    # 삭제하기
    result: bool = Database.delete_message(message_id)

    if result:
        return {
            "message": "Message deleted successfully"
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to delete message"
            }
        )
