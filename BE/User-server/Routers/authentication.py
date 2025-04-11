"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Parts of Members
"""

# Libraries
from fastapi import HTTPException, APIRouter, status, Response, Request, Depends
from fastapi.encoders import jsonable_encoder

import Database
from Database.models import *

from Endpoint.models import *

from Utilities.auth_tools import *
from Utilities.logging_tools import *

import os
from dotenv import load_dotenv
from typing import Literal

# 개발 및 배포 서버 여부 확인
load_dotenv()
isDev: bool = bool(int(os.getenv("IS_DEV", 0)))
isDeploy: bool = bool(int(os.getenv("IS_DEPLOY", 0)))
SECURE_SET: bool = True
SAME_SET: Literal["lax", "strict", "none"] = "none"
DOMAIN_SET: str = ".itdice.net" if isDeploy else "localhost"

router = APIRouter(prefix="/auth", tags=["Authentication"])
logger = get_logger("Router_Authentication")

# ========== Auth 부분 ==========

# 사용자가 로그인하는 기능
@router.post("/login", status_code=status.HTTP_200_OK)
async def login(response: Response, login_data: Login):
    # 필요한 정보가 입력되었는지 점검
    missing_location: list = ["body"]

    if login_data.email is None or login_data.email == "":
        missing_location.append("email")
    if login_data.password is None or login_data.password == "":
        missing_location.append("password")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Login data is required",
                "input": {"email": login_data.email, "password": "<PASSWORD>"}
            }
        )

    # 사용자의 이메일을 이용해 ID를 가져오기
    user_id: str = Database.get_id_from_email(login_data.email)

    if not user_id:
        logger.warning(f"Invalid email or password: {login_data.email}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "type": "unauthorized",
                "message": "Invalid email or password",
                "input": {"email": login_data.email, "password": "<PASSWORD>"}
            }
        )

    # 사용자의 기본 정보 가져오기
    user_data: dict = Database.get_one_account(user_id)

    if not user_data:
        logger.warning(f"Invalid email or password: {login_data.email}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "type": "unauthorized",
                "message": "Invalid email or password",
                "input": {"email": login_data.email, "password": "<PASSWORD>"}
            }
        )

    # 비밀번호 검증
    input_password: str = login_data.password
    hashed_password: str = Database.get_hashed_password(user_id)
    is_verified: bool = verify_password(input_password, hashed_password)

    if not is_verified:
        logger.warning(f"Invalid email or password: {login_data.email}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "type": "unauthorized",
                "message": "Invalid email or password",
                "input": {"email": login_data.email, "password": "<PASSWORD>"}
            }
        )

    # Session 생성하기
    new_xid: str = random_xid(16)

    new_session: LoginSessionsTable = LoginSessionsTable(
        xid=new_xid,
        user_id=user_id,
        is_main_user=user_data["role"] is Role.MAIN,
        is_remember=False
    )

    # Sesstion 생성하기
    result: bool = Database.create_session(new_session)

    if not result:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to create session",
                "input": {"email": login_data.email, "password": "<PASSWORD>"}
            }
        )

    # 식별용 쿠키 설정
    response.set_cookie(
        key="session_id",
        value=new_xid,
        httponly=True,
        secure=SECURE_SET,
        samesite=SAME_SET,
        domain=DOMAIN_SET,
    )

    logger.info(f">>> Login successful: {new_xid} <<<")

    return {
        "message": "Login successful",
        "result": {
            "session_id": new_xid,
            "user_data": jsonable_encoder(user_data)
        }
    }

# 사용자가 로그아웃 하는 기능
@router.post("/logout", status_code=status.HTTP_200_OK)
async def logout(request: Request, response: Response):
    session_id: str = request.cookies.get("session_id")
    session_data: dict = Database.get_login_session(session_id)

    if session_id and session_data:
        # Session 삭제하기
        result: bool = Database.delete_session(session_id)

        if not result:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={
                    "type": "server error",
                    "message": "Failed to delete session",
                    "input": {"session_id": session_id}
                }
            )

        # 식별용 쿠키 삭제
        response.delete_cookie("session_id")

    logger.info(f">>> Logout successful: {session_id} <<<")

    return {
        "message": "Logout successful"
    }

# 사용자의 비밀번호를 변경하는 기능
@router.patch("/change-password", status_code=status.HTTP_200_OK)
async def change_password(change_password_data: ChangePassword, request_id: str = Depends(Database.check_current_user)):
    target_user_id: str = change_password_data.user_id

    # 필수 입력 정보를 전달했는지 점검
    missing_location: list = ["body"]

    if change_password_data.user_id is None or change_password_data.user_id == "":
        missing_location.append("user_id")
    if change_password_data.current_password is None or change_password_data.current_password == "":
        missing_location.append("current_password")
    if change_password_data.new_password is None or change_password_data.new_password == "":
        missing_location.append("new_password")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Password data is required",
                "input": {
                    "user_id": target_user_id,
                    "current_password": "<PASSWORD>",
                    "new_password": "<PASSWORD>"
                }
            }
        )

    # 시스템 계정을 제외한 사용자는 자신의 계정 비밀번호만 변경할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and target_user_id != request_id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": {
                    "user_id": change_password_data.user_id,
                    "current_password": "<PASSWORD>",
                    "new_password": "<PASSWORD>"
                }
            }
        )

    # 존재하는 사용자인지 확인
    user_data: dict = Database.get_one_account(target_user_id)

    if not user_data:
        logger.warning(f"User not found: {target_user_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "User not found",
                "input": {
                    "user_id": target_user_id,
                    "current_password": "<PASSWORD>",
                    "new_password": "<PASSWORD>"
                }
            }
        )

    if request_data["role"] is not Role.SYSTEM:
        # 현재 비밀번호 검증
        input_current_password: str = change_password_data.current_password
        hashed_current_password: str = Database.get_hashed_password(target_user_id)
        is_verified_current: bool = verify_password(input_current_password, hashed_current_password)

        if not is_verified_current:
            logger.warning(f"Invalid password: {target_user_id}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "type": "unauthorized",
                    "message": "Invalid password",
                    "input": {
                        "user_id": target_user_id,
                        "current_password": "<PASSWORD>",
                        "new_password": "<PASSWORD>"
                    }
                }
            )

    # 새로운 비밀번호로 설정
    new_password: str = change_password_data.new_password
    hashed_new_password: str = hash_password(new_password)
    result: bool = Database.change_password(target_user_id, hashed_new_password)

    if result:
        return {
            "message": "Password changed successfully"
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to change password",
                "input": {
                    "user_id": target_user_id,
                    "current_password": "<PASSWORD>",
                    "new_password": "<PASSWORD>"
                }
            }
        )

# 자동 로그인을 설정하는 기능
@router.patch("/auto-login", status_code=status.HTTP_200_OK)
async def set_auto_login(user_request: Request, request_id: str = Depends(Database.check_current_user)):
    # 보낸 Session 정보가 정상적인지 검증하기
    session_id: str = user_request.cookies.get("session_id")
    session_data: dict = Database.get_login_session(session_id)

    if not session_id or not request_id or not session_data:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 자동 로그인 사용 처리하기
    result: bool = Database.record_auto_login(session_id)

    if result:
        logger.info(f">>> Auto login set successful: {session_id} <<<")
        return {
            "message": "Auto login set successfully"
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to set auto login"
            }
        )

# 지금 유효한 권한을 가지고 있는지 확인하는 기능
@router.post("/check", status_code=status.HTTP_200_OK)
async def check_permission(session_data: SessionCheck, response: Response):
    missing_location: list = ["body"]

    if session_data.session_id is None or session_data.session_id == "":
        missing_location.append("session_id")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Permission check data is required",
                "input": jsonable_encoder(session_data)
            }
        )

    # 보낸 세션 ID가 유효한지 확인
    session_id: str = session_data.session_id
    session_data: dict = Database.get_login_session(session_id)
    request_id: str = session_data["user_id"] if session_data else None
    request_data: dict = Database.get_one_account(request_id)

    if not session_data or not request_data:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 식별용 쿠키 설정
    response.set_cookie(
        key="session_id",
        value=session_id,
        httponly=True,
        secure=SECURE_SET,
        samesite=SAME_SET,
        domain=DOMAIN_SET,
    )

    logger.info(f">>> Re-Login successful: {session_id} <<<")

    return {
        "message": "Permission check successful",
        "result": {
            "session_id": session_id,
            "user_data": jsonable_encoder(request_data)
        }
    }
