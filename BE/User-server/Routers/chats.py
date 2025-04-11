"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Parts of Chats
"""

# Libraries
from fastapi import HTTPException, APIRouter, status, Depends
from fastapi.encoders import jsonable_encoder

import httpx

import Database
from Database.models import *
from External.ai import talk_with_ai

from Utilities.check_tools import *
from Utilities.logging_tools import *

router = APIRouter(prefix="/chats", tags=["Chats"])
logger = get_logger("Router_Chats")

# ========== Chats 부분 ==========

# AI와 채팅을 진행하는 기능
@router.post("", status_code=status.HTTP_200_OK)
async def chat_with_ai(chat_data: AIChat, request_id = Depends(Database.check_current_user)):
    # 필수 입력 정보를 전달했는지 점검
    missing_location: list = ["body"]

    if chat_data.user_id is None or chat_data.user_id == "":
        missing_location.append("user_id")
    if chat_data.message is None or chat_data.message == "":
        missing_location.append("message")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Password data is required",
                "input": {
                    "user_id": chat_data.user_id,
                    "message": chat_data.message,
                    "session_id": chat_data.session_id
                }
            }
        )

    # 시스템 계정을 제외하고 주 사용자만 AI Chat 기능을 사용할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and
                            (request_data["role"] != Role.MAIN or request_id != chat_data.user_id)):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": {
                    "user_id": chat_data.user_id,
                    "message": chat_data.message,
                    "session_id": chat_data.session_id
                }
            }
        )

    # AI에게 Chat 보내기
    response: httpx.Response = await talk_with_ai(
        user_id=chat_data.user_id,
        message=chat_data.message,
        session_id=chat_data.session_id
    )

    if response is not None and response.status_code == status.HTTP_200_OK:
        logger.info(f"Chat with AI successfully: {chat_data.user_id}")
        return {
            "message": "Chat sent successfully",
            "result": jsonable_encoder(response.json())
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to send chat",
                "input": {
                    "user_id": chat_data.user_id,
                    "message": chat_data.message,
                    "session_id": chat_data.session_id
                }
            }
        )
