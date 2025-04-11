"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
version : 0.1.0
"""

# Libraries
from fastapi import FastAPI, UploadFile, File, HTTPException, status, Request, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from contextlib import asynccontextmanager

import filetype

import Database
from Database.models import *

from datetime import datetime, timezone

import os
from dotenv import load_dotenv

from Utilities.logging_tools import *

logger = get_logger("Image")


# ========== 백그라운드 기능 ==========
@asynccontextmanager
async def startup(app: FastAPI):
    # 시작된 경우
    logger.info("🚀 Start Care-bot Image Provider!!!")

    yield

    # 종료 된 경우
    logger.info("🛑 Server shutdown")


app = FastAPI(lifespan=startup)

# ========== CORS 설정 ==========
origins_url = [
    "http://localhost:3000",
    "http://localhost:8080",
    "http://127.0.0.1:3000",
    "http://127.0.0.1:8080",
    "https://dev-main.itdice.net",
    "https://dev-sub.itdice.net",
    "https://main.itdice.net",
    "https://sub.itdice.net",
    "https://dev-ai.itdice.net",
    "https://ai.itdice.net",
    "https://dev-api.itdice.net",
    "https://api.itdice.net"
]

app.add_middleware(  # type: ignore
    CORSMiddleware,
    allow_origins=origins_url,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ========== 이미지 저장공간 설정 ==========
load_dotenv()

image_url: str = os.getenv("IMAGE_URL")
image_storage: str = os.getenv("IMAGE_STORAGE")
os.makedirs(image_storage, exist_ok=True)
checker_size: int = 2048
allowed_types: list[str] = ["image/jpeg", "image/png", "image/gif", "image/webp"]
max_image_size: int = int(os.getenv("MAX_IMAGE_SIZE")) * 1024 * 1024
cache_duration: int = int(os.getenv("CACHE_DURATION"))


# ========== 이미지 관리 기능 ==========
@app.post("/upload", status_code=status.HTTP_201_CREATED)
async def upload_image(request: Request, file: UploadFile = File(...), request_id=Depends(Database.check_current_user)):
    # 사용자 계정을 통해 접근하는지 점검
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"Can not access image: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 파일 크기 검사
    content_length = request.headers.get("Content-Length")
    if content_length is not None and int(content_length) > max_image_size:
        raise HTTPException(
            status_code=status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
            detail={
                "type": "too large",
                "message": f"Image size exceeds the limit({max_image_size // 1024 // 1024}MB max).",
                "input": {
                    "request_id": request_id,
                    "file_name": file.filename,
                    "file_size": f"{int(content_length) // 1024 // 1024}MB",
                }
            }
        )

    # 파일 내용 검사
    file_bytes = await file.read(checker_size)
    file_type = filetype.guess(file_bytes)

    if file_type is None or file_type.mime not in allowed_types:
        logger.warning(f"Invalid file type: {file_type.mime if file_type else 'unknown'}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "type": "invalid value",
                "message": "Only image files can be uploaded",
                "input":{
                    "request_id": request_id,
                    "file_name": file.filename,
                    "file_type": file_type.mime if file_type else 'unknown'
                }
            }
        )

    file.file.seek(0)  # 포인터 초기화

    # 해당 사용자의 이미지 저장공간 접근
    try:
        user_storage = os.path.join(image_storage, request_id)
        os.makedirs(user_storage, exist_ok=True)

        # 파일 이름 설정
        filename, ext = os.path.splitext(file.filename)
        current_datatime = datetime.now(tz=timezone.utc)
        time_header = current_datatime.strftime("%Y%m%d_%H%M%S") + f"_{current_datatime.microsecond // 1000:03d}"
        new_filename = f"{filename}_{time_header}{ext}"

        # 파일 저장 경로
        file_path = os.path.join(user_storage, new_filename)

        # 파일 저장
        with open(file_path, "wb") as _buffer:
            content = await file.read()
            _buffer.write(content)

        logger.info(f"Image uploaded: {new_filename}")
        file.file.seek(0)  # 포인터 초기화

        return {
            "message": "Image uploaded successfully",
            "result": {
                "request_id": request_id,
                "file_name": new_filename,
                "file_type": file_type.mime if file_type else 'unknown',
                "file_path": f"{image_url}/access/{request_id}/{new_filename}"
            }
        }
    except Exception as error:
        logger.error(f"Image upload failed: {str(error)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "internal server error",
                "message": "An error occurred while uploading the image"
            }
        )


@app.get("/access/{user_id}/{file_name}", status_code=status.HTTP_200_OK)
async def get_image(user_id: str, file_name: str, request_id=Depends(Database.check_current_user)):
    # 사용자 계정을 통해 접근하는지 확인
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning(f"Can not access image: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 접근 권한 범위 설정
    accessible_id: list[str] = [request_id]

    if request_data["role"] == Role.MAIN:  # 주 사용자가 접근한 경우 소속된 가족 이미지까지 접근 가능
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

    # 요청한 사용자가 해당 이미지 경로에 접근 가능한지 점검
    if request_data["role"] != Role.SYSTEM and user_id not in accessible_id:
        logger.warning(f"Can not access image: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 해당 파일이 존재하는지 확인
    file_path = os.path.join(image_storage, user_id, file_name)

    if not os.path.exists(file_path):
        logger.warning(f"Image not found: {file_path}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Image not found"
            }
        )

    # 파일 종류 감지하기
    file_type = None

    with open(file_path, "rb") as buffer:
        file_bytes = buffer.read(checker_size)
        file_type = filetype.guess(file_bytes)

    if file_type is None or file_type.mime not in allowed_types:
        logger.error(f"Not image file: {file_type.mime if file_type else 'unknown'}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "type": "invalid value",
                "message": "Only image files can be provided to the user",
                "input": {
                    "request_id": request_id,
                    "file_name": file_name,
                }
            }
        )

    try:
        cache_headers = {"Cache-Control": f"public, max-age={cache_duration}"}

        return FileResponse(file_path, media_type=file_type.mime, headers=cache_headers)
    except Exception as error:
        logger.error(f"Image access failed: {str(error)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "internal server error",
                "message": "An error occurred while accessing the image"
            }
        )


@app.delete("/delete/{user_id}/{file_name}", status_code=status.HTTP_200_OK)
async def delete_image(user_id: str, file_name: str, request_id=Depends(Database.check_current_user)):
    # 시스템 계정을 제외한 이미지의 소유자만 삭제할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and user_id != request_id):
        logger.warning(f"Can not delete image: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 해당 파일이 존재하는지 점검
    file_path = os.path.join(image_storage, user_id, file_name)

    if not os.path.exists(file_path):
        logger.warning(f"Image not found: {file_path}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Image not found"
            }
        )

    try:
        # 파일 삭제하기
        os.remove(file_path)

        return {
            "message": "Image deleted successfully",
            "result": {
                "request_id": request_id,
                "file_name": file_name
            }
        }
    except Exception as error:
        logger.error(f"Image deletion failed: {str(error)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "internal server error",
                "message": "An error occurred while deleting the image"
            }
        )
