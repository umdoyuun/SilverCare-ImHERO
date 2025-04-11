"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Parts of Families
"""

# Libraries
from fastapi import HTTPException, APIRouter, status, Depends
from fastapi.encoders import jsonable_encoder

import Database
from Database.models import *

from Utilities.auth_tools import *
from Utilities.check_tools import *
from Utilities.logging_tools import *

from datetime import date

router = APIRouter(prefix="/families", tags=["Families"])
logger = get_logger("Router_Families")

# ========== Family 부분 ==========

# 주 사용자의 ID를 이용해 가족이 이미 생성되었는지 확인하는 기능
@router.post("/check-exist", status_code=status.HTTP_200_OK)
async def check_family_from_main_id(family_check: IDCheck, request_id: str = Depends(Database.check_current_user)):
    # 필수 입력 조건 점검
    missing_location: list = ["body"]

    if family_check.id is None or family_check.id == "":
        missing_location.append("id")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": ["body", "id"],
                "message": "Main ID is required",
                "input": jsonable_encoder(family_check)
            }
        )

    # 시스템 계정을 제외하고 확인하려는 주 사용자 본인만 확인할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id != family_check.id):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": jsonable_encoder(family_check)
            }
        )

    # 가족 정보가 존재하는지 확인
    exist_family: str = Database.main_id_to_family_id(family_check.id)

    if exist_family == "":
        logger.warning(f"Main ID does not have a family: {family_check.id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Main ID does not have a family",
                "input": jsonable_encoder(family_check)
            }
        )
    else:
        logger.info(f"Main ID has a family: {family_check.id}")
        return {
            "message": "Family exists",
            "result": {"family_id": exist_family}
        }

# 주 사용자를 기반으로 새로운 가족을 생성하는 기능
@router.post("", status_code=status.HTTP_201_CREATED)
async def create_family(family_data: Family, request_id: str = Depends(Database.check_current_user)):
    # 필수 입력 정보 점검
    missing_location: list = ["body"]

    if family_data.main_user is None or family_data.main_user == "":
        missing_location.append("main_user")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Main user ID is required",
                "input": jsonable_encoder(family_data)
            }
        )

    # 시스템 계정을 제외하고 확인하려는 주 사용자 본인만 가족을 생성할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data or (request_data["role"] != Role.SYSTEM and request_id != family_data.main_user):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": jsonable_encoder(family_data)
            }
        )

    # 가족이 이미 생성되었는지 점검
    exist_family: str = Database.main_id_to_family_id(family_data.main_user)

    if exist_family:
        logger.warning(f"Main user already has a family: {family_data.main_user}")
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={
                "type": "already exists",
                "message": "Main user already has a family",
                "input": jsonable_encoder(family_data)
            }
        )

    # 주 사용자 존재 확인 및 역할 점검
    exist_user: dict= Database.get_one_account(family_data.main_user)

    if not exist_user or exist_user["role"] is not Role.MAIN:
        logger.warning(f"Invalid User: {family_data.main_user}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "type": "invalid value",
                "message": "You are not a main user",
                "input": jsonable_encoder(family_data)
            }
        )

    # ID 생성 및 중복 점검
    new_id: str = ""
    id_verified: bool = False

    while not id_verified:
        new_id = random_id(16, Identify.FAMILY)
        families: list = Database.get_all_families()
        id_list = [data["id"] for data in families]
        if new_id not in id_list:
            id_verified = True

    # 새로운 가족 정보 생성
    new_family: FamiliesTable = FamiliesTable(
        id=new_id,
        main_user=family_data.main_user,
        family_name=family_data.family_name
    )

    # 새로운 가족 생성
    result: bool = Database.create_family(new_family)

    if result:
        # Settings 값 생성
        settings_data: SettingsTable = SettingsTable(family_id=new_id)
        settings_result: bool = Database.create_settings(settings_data)

        if not settings_result:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={
                    "type": "server error",
                    "message": "Failed to create new settings",
                    "input": jsonable_encoder(family_data)
                }
            )

        return {
            "message": "New family created successfully",
            "result": {
                "id": new_id
            }
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to create new family",
                "input": jsonable_encoder(family_data)
            }
        )

# 모든 가족의 정보를 불러오는 기능
@router.get("", status_code=status.HTTP_200_OK)
async def get_all_families(request_id: str = Depends(Database.check_current_user)):
    # 시스템 관리자만 접근할 수 있음
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

    # 가족 정보 불러오기
    family_list: list = Database.get_all_families()

    if family_list:
        return {
            "message": "All families retrieved successfully",
            "result": jsonable_encoder(family_list)
        }
    else:
        logger.warning("No families found.")
        return {
            "message": "No families found",
            "result": jsonable_encoder(family_list)
        }

@router.post("/find", status_code=status.HTTP_200_OK)
async def find_family(find_data: FindFamily, request_id: str = Depends(Database.check_current_user)):
    # 필수 입력 정보 점검
    missing_location: list = ["body"]

    if find_data.user_name is None or find_data.user_name == "":
        missing_location.append("user_name")
    if find_data.birth_date is None:
        missing_location.append("birth_date")
    if find_data.birth_date.year is None or find_data.birth_date.year == 0:
        missing_location.append("birth_date.year")
    if find_data.birth_date.month is None or find_data.birth_date.month == 0:
        missing_location.append("birth_date.month")
    if find_data.birth_date.day is None or find_data.birth_date.day == 0:
        missing_location.append("birth_date.day")
    if find_data.gender is None:
        missing_location.append("gender")
    if find_data.address is None or find_data.address == "":
        missing_location.append("address")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Required fields are missing",
                "input": jsonable_encoder(find_data)
            }
        )

    # 계정이 있는 사용자가 가족을 검색할 수 있음
    request_data: dict = Database.get_one_account(request_id)

    if not request_data:
        logger.warning("No User")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission",
                "input": jsonable_encoder(find_data)
            }
        )

    # 잘못된 성별을 선택했는지 점검
    if find_data.gender is not None and find_data.gender.lower() not in Gender._value2member_map_:
        logger.error(f"Invalid gender: {find_data.gender}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "type": "invalid value",
                "message": "Invalid value provided for find option (gender)",
                "input": jsonable_encoder(find_data)
            }
        )

    # 성별 변환하기
    find_gender: Gender = Gender(find_data.gender)

    # 날짜 변환하기
    find_birthday = None
    if find_data.birth_date is not None:
        if is_valid_date(find_data.birth_date) is False:
            logger.error(f"Invalid birth date: {find_data.birth_date}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "type": "invalid value",
                    "message": "Invalid value provided for find option (birth_date)",
                    "input": jsonable_encoder(find_data)
                }
            )
        else:
            find_birthday = date(
                year=find_data.birth_date.year,
                month=find_data.birth_date.month,
                day=find_data.birth_date.day
            )

    # 가족 찾기
    result: list[dict] = Database.find_family(
        user_name=find_data.user_name,
        birth_date=find_birthday,
        gender=find_gender,
        address=find_data.address,
    )

    if result:
        return {
            "message": "Families found successfully",
            "result": jsonable_encoder(result)
        }
    else:
        logger.warning("No family found.")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "No families found",
                "input": jsonable_encoder(find_data)
            }
        )

# 가족 정보를 불러오는 기능
@router.get("/{family_id}", status_code=status.HTTP_200_OK)
async def get_family(family_id: str, request_id: str = Depends(Database.check_current_user)):
    # 시스템 계정을 제외한 가족의 주 사용자, 보조 사용자만 가족 정보를 불러올 수 있음
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

    # 가족 정보 불러오기
    family_data: dict = Database.get_one_family(family_id)

    if family_data:
        return {
            "message": "Family retrieved successfully",
            "result": jsonable_encoder(family_data)
        }
    else:
        logger.warning(f"Family not found: {family_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Family not found",
                "input": {"family_id": family_id}
            }
        )

@router.get("/name/{family_id}", status_code=status.HTTP_200_OK)
async def get_name_family(family_id: str, request_id: str = Depends(Database.check_current_user)):
    # 계정이 있는 사용자가 가족의 이름을 조회해볼 수 있음
    request_data: dict = Database.get_one_account(request_id);

    if not request_data:
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 가족의 정보를 불러오기
    family_data: dict = Database.get_one_family(family_id)

    if family_data:
        return {
            "message": "Family retrieved successfully",
            "result": {"family_name": family_data["family_name"] if family_data["family_name"] else None}
        }
    else:
        logger.warning(f"Family not found: {family_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "type": "not found",
                "message": "Family not found"
            }
        )

# 가족 정보를 수정하는 기능 (family_name만 수정 가능)
@router.patch("/{family_id}", status_code=status.HTTP_200_OK)
async def update_family(family_id: str, updated_family: Family, request_id: str = Depends(Database.check_current_user)):
    # 시스템 계정을 제외한 가족의 주 사용자, 보조 사용자만 가족의 정보를 수정할 수 있음
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

    # 최종적으로 변경할 데이터 생성
    total_updated_family: FamiliesTable = FamiliesTable(
        id=family_id,
        main_user=family_data["main_user"],
        family_name=updated_family.family_name if updated_family.family_name is not None else family_data["family_name"]
    )

    # 가족 정보 변경
    result: bool = Database.update_one_family(family_id, total_updated_family)

    if result:
        final_updated_family: dict = Database.get_one_family(family_id)
        return {
            "message": "Family updated successfully",
            "result": jsonable_encoder(final_updated_family)
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to update family",
                "input": jsonable_encoder(updated_family)
            }
        )

# 가족 정보를 삭제하는 기능 (주 사용자의 비밀번호 필요)
@router.delete("/{family_id}", status_code=status.HTTP_200_OK)
async def delete_family(family_id: str, checker: PasswordCheck, request_id: str = Depends(Database.check_current_user)):
    # 필수 입력 조건 점검
    missing_location: list = ["body"]

    if checker.password is None or checker.password == "":
        missing_location.append("password")

    if len(missing_location) > 1:
        logger.error(f"No data provided: {missing_location}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={
                "type": "no data",
                "loc": missing_location,
                "message": "Password is required",
                "input": {"family_id": family_id, "password": "<PASSWORD>"}
            }
        )

    # 시스템 계정을 제외하고 확인하려는 주 사용자 본인만 가족을 생성할 수 있음
    request_data: dict = Database.get_one_account(request_id)
    family_data: dict = Database.get_one_family(family_id)

    if not request_data or not family_data or (request_data["role"] != Role.SYSTEM and request_id != family_data["main_user"]):
        logger.warning(f"You do not have permission: {request_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "type": "can not access",
                "message": "You do not have permission"
            }
        )

    # 비밀번호 검증
    if request_data["role"] is not Role.SYSTEM:
        input_password: str = checker.password
        hashed_password: str = Database.get_hashed_password(family_data["main_user"])
        is_verified: bool = verify_password(input_password, hashed_password)

        if not is_verified:
            logger.warning(f"Invalid password: {family_id}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "type": "unauthorized",
                    "message": "Invalid password",
                    "input": {"family_id": family_id, "password": "<PASSWORD>"}
                }
            )

    # 가족 삭제 진행
    final_result: bool = Database.delete_one_family(family_id)

    if final_result:
        return {
            "message": "Family deleted successfully"
        }
    else:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "type": "server error",
                "message": "Failed to delete family",
                "input": {"family_id": family_id, "password": "<PASSWORD>"}
            }
        )