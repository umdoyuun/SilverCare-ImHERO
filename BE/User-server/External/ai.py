"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
External Server Connection
"""
# Libraries
import httpx
from datetime import datetime, timezone

from Utilities.logging_tools import *

import os
from dotenv import load_dotenv

logger = get_logger("External_AI")

# 외부 AI Process 서버 확인
load_dotenv()
isDeploy: bool = bool(int(os.getenv("IS_DEPLOY", 0)))
AI_HOST: str = os.getenv("AI_HOST") if isDeploy else "http://localhost"
AI_PORT: int = int(os.getenv("AI_PORT"))
AI_PATH: str = f"{AI_HOST}:{AI_PORT}"
external_timeout: float = float(os.getenv("EXTERNAL_TIMEOUT", 60.0))
set_timeout = httpx.Timeout(timeout=external_timeout)

# ========== Heartbeat ==========

# AI Process 상태 확인기능
async def check_connection() -> httpx.Response | None:
    """
    AI Process의 상태를 확인하는 기능
    """
    external_url = f"{AI_PATH}/heartbeat"

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.get(external_url)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to check connection with AI server: {str(error)}")
        return None

# ========== Status 부분 ==========

# 단일 정신건강 정보를 요청하는 기능
async def request_mental_status(family_id: str) -> httpx.Response | None:
    """
    AI Server에게 정신 건강 정보를 요청하는 기능
    :param family_id: 해당하는 가족의 ID
    :return: AI Server로부터 반환된 결과 값 (httpx.Response)
    """
    external_url = f"{AI_PATH}/generate-emotional-report/{family_id}"

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.post(external_url)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to request mental status from AI server: {str(error)}")
        return None

# 날짜 범위로 정신 건강 리포트를 요청하는 기능
async def request_mental_reports(family_id: str, start: datetime, end: datetime) -> httpx.Response | None:
    """
    AI Server에게 날짜 범위에 해당하는 정신 건강 리포트를 요청하는 기능
    :param family_id: 해당하는 가족의 ID
    :param start: 리포트를 생성할 데이터의 시작 날짜 및 시각
    :param end: 리포트를 생성할 데이터의 마지막 날짜 및 시각
    :return: AI Server로부터 반환된 결과 값 (httpx.Response)
    """
    external_url = f"{AI_PATH}/generate-emotional-report/period/{family_id}"

    request_data = {
        "start_date": start.isoformat(timespec='seconds'),
        "end_date": end.isoformat(timespec='seconds')
    }

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.post(external_url, json=request_data)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to request mental reports from AI server: {str(error)}")
        return None

# 하루의 대화 키워드를 요약하는 기능
async def request_conversation_keywords(family_id: str) -> httpx.Response | None:
    """
    주 사용자의 AI Chat 기록을 기반으로 대화 키워드를 요약하는 기능
    :param family_id: 해당하는 가족의 ID
    :return: AI Server로부터 반환된 결과 값 (httpx.Response)
    """
    external_url = f"{AI_PATH}/generate-keyword/{family_id}"

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.get(external_url)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to request conversation keywords from AI server: {str(error)}")
        return None

# 종합적인 심리 보고서를 요청하는 기능
async def request_psychology_report(
        family_id: str,
        start: datetime,
        end: datetime = datetime.now(tz=timezone.utc)) -> httpx.Response | None:
    """
    주 사용자의 AI Chat 기록을 기반으로 심리 상태 보고서를 생성하는 기능 (DB 저장 X)
    :param family_id: 해당하는 가족의 ID
    :param start: 보고서를 생성할 데이터의 시작 날짜 및 시각
    :param end: 보고서를 생성할 데이터의 마지막 날짜 및 시각
    :return: AI Server로부터 반환된 결과 값 (httpx.Response)
    """
    external_url = f"{AI_PATH}/analyze-mental-health/{family_id}"

    request_data = None
    if start is not None:
        request_data = {
            "start_date": start.isoformat(timespec='seconds') if start else None,
            "end_date": end.isoformat(timespec='seconds') if end else None
        }

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.post(external_url, json=request_data)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to request psychology report from AI server: {str(error)}")
        return None

# ========== Chats 부분 ==========

# AI와 채팅하기 위해 메시지를 보내는 기능
async def talk_with_ai(user_id: str, message: str, session_id: str = None) -> httpx.Response | None:
    """
    AI와 채팅을 위해 메시지를 보내는 기능
    :param user_id: 채팅을 보내는 사용자의 ID
    :param message: 채팅 내용
    :param session_id: 이전 대화 내용을 확인하기 위한 Session ID
    :return: AI Server로부터 전달받은 AI의 답장 (httpx.Response)
    """
    external_url = f"{AI_PATH}/chat"

    request_data = {
        "user_id": user_id,
        "user_message": message,
        "session_id": session_id if session_id else None
    }

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.post(external_url, json=request_data)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to talk with AI server: {str(error)}")
        return None

# ========== News & Weather 부분 ==========

# >>> Deprecated <<<
# 한국의 최신 뉴스를 불러오는 기능
async def korean_news() -> httpx.Response | None:
    external_url = f"{AI_PATH}/news"

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.get(external_url)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to get news from AI server: {str(error)}")
        return None

# 사용자의 위치의 날씨 정보를 가져오는 기는
async def korean_weather(user_id: str) -> httpx.Response | None:
    external_url = f"{AI_PATH}/weather/{user_id}"

    try:
        async with httpx.AsyncClient(timeout=set_timeout) as client:
            response = await client.get(external_url)
            return response
    except httpx.RequestError as error:
        logger.critical(f"Error: Unable to get weather from AI server: {str(error)}")
        return None
