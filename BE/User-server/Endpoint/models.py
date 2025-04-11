"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Endpoint Data Models
"""

# Libraries
from pydantic import BaseModel, field_validator
from typing import Optional
from datetime import datetime

class Date(BaseModel):
    """
    날짜 데이터
    """
    year: int
    month: int
    day: int

class EmailCheck(BaseModel):
    """
    중복 이메일을 검증하기 위해 Client가 보내는 데이터
    """
    email: Optional[str] = None  # 중복을 검증할 이메일 주소

class PasswordCheck(BaseModel):
    """
    계정 삭제를 위해 Client가 보내는 데이터
    """
    password: Optional[str] = None  # 계정 삭제를 위해 검증할 비밀번호

class Account(BaseModel):
    """
    계정 생성을 위해 client가 보내는 데이터
    """
    email: Optional[str] = None  # 로그인에 사용할 이메일 주소
    password: Optional[str] = None  # 로그인에 사용할 비밀번호
    role: Optional[str] = None  # 사용자의 역할 ("main", "sub", "system", "test")
    user_name: Optional[str] = None  # 사용자 본명
    birth_date: Optional[Date] = None  # 사용자 생년월일
    gender: Optional[str] = None  # 사용자 성별 ("male" or "female")
    address: Optional[str] = None  # 사용자 거주지 ("읍면동" 단위까지)

    @field_validator("user_name")
    def trim_user_name(cls, value):
        if value and len(value) > 32:
            return value[:32]
        return value

    @field_validator("address")
    def trim_address(cls, value):
        if value and len(value) > 128:
            return value[:128]
        return value

class IDCheck(BaseModel):
    """
    계정 확인 및 점검을 위해 Client가 보내는 데이터
    """
    id: Optional[str] = None  # 확인 및 점검을 위한 ID

class SessionCheck(BaseModel):
    """
    Session이 유효한지 확인하기 위해 Client가 보내는 데이터
    """
    session_id: Optional[str] = None  # 재 로그인을 위한 Session ID

class Family(BaseModel):
    """
    가족 생성을 위해 client가 보내는 데이터
    """
    main_user: Optional[str] = None  # 가족의 대표가 될 주 사용자 ID
    family_name: Optional[str] = None  # 가족 구성의 별명

    @field_validator("family_name")
    def trim_family_name(cls, value):
        if value and len(value) > 32:
            return value[:32]
        return value

class FindFamily(BaseModel):
    """
    해당하는 가족을 찾기위해 client가 보내는 데이터
    """
    user_name: Optional[str] = None  # 사용자 이름
    birth_date: Optional[Date] = None  # 사용자의 생년월일
    gender: Optional[str] = None  # 사용자의 성별 ("male" or "female")
    address: Optional[str] = None  # 사용자의 거주지

class Member(BaseModel):
    """
    가족 관계 생성을 위해 client가 보내는 데이터
    """
    family_id: Optional[str] = None  # 가족의 ID
    user_id: Optional[str] = None  # 관계를 생성할 대상의 사용자 ID
    nickname: Optional[str] = None  # 주 사용자에게 보여질 별명

    @field_validator("nickname")
    def trim_nickname(cls, value):
        if value and len(value) > 32:
            return value[:32]
        return value

class Login(BaseModel):
    """
    로그인을 위해 client가 보내는 데이터
    """
    email: Optional[str] = None  # 로그인에 사용할 이메일 주소
    password: Optional[str] = None  # 로그인에 사용할 비밀번호

class ChangePassword(BaseModel):
    """
    사용자의 비밀번호를 변경하기 위해서 client가 보내는 데이터
    """
    user_id: Optional[str] = None  # 사용자의 ID
    current_password: Optional[str] = None  # 기존의 비밀번호
    new_password: Optional[str] = None  # 새로운 비밀번호

class HomeStatus(BaseModel):
    """
    Carebot 장치에서 집안의 환경 상태를 보고하는 데이터
    """
    family_id: Optional[str] = None  # 가족의 ID
    temperature: Optional[float] = None  # 집안의 온도
    humidity: Optional[float] = None  # 집안의 습도
    dust_level: Optional[float] = None  # 집안의 미세먼지 농도
    ethanol: Optional[float] = None  # 집안의 에탄올 수치 (GAS 센서 대응)
    others: Optional[str] = None  # 그 외 추가 데이터나 설명

class HealthStatus(BaseModel):
    """
    Carebot 장치에서 건강 상태를 보고하는 데이터
    """
    family_id: Optional[str] = None  # 가족의 ID
    heart_rate: Optional[float] = None  # 분당 심장박동수

class ActiveStatus(BaseModel):
    """
    Carebot 장치에서 활동 상태를 보고하는 데이터
    """
    family_id: Optional[str] = None  # 가족의 ID
    score: Optional[int] = None  # 활동 점수
    action: Optional[str] = None  # 활동에 대한 간략한 요약
    is_critical: Optional[bool] = None  # 활동이 상태에 치명적인지 여부
    description: Optional[str] = None  # 상세한 보고 내용
    image_url: Optional[str] = None  # 상태에 대한 이미지

class AIChat(BaseModel):
    """
    AI와 대화를 위해 Client가 보내는 데이터
    """
    user_id: Optional[str] = None
    message: Optional[str] = None
    session_id: Optional[str] = None

class Notification(BaseModel):
    """
    알림을 생성하기 위해 Client가 보내는 데이터
    """
    family_id: Optional[str] = None
    notification_grade: Optional[str] = None
    descriptions: Optional[str] = None
    image_url: Optional[str] = None

class Message(BaseModel):
    """
    사용자간의 메시지를 주고 받기 위해 Client가 보내는 데이터
    """
    from_id: Optional[str] = None
    to_id: Optional[str] = None
    content: Optional[str] = None
    image_url: Optional[str] = None

class IndexList(BaseModel):
    """
    여러 메시지, 알림을 읽음 처리하기 위해 Client가 보내는 데이터
    """
    index_list: Optional[list] = None

class Settings(BaseModel):
    """
    로봇의 설정 값을 업데이트하기 위해 Client가 보내는 데이터
    """
    is_alarm_enabled: Optional[bool] = None
    is_camera_enabled: Optional[bool] = None
    is_microphone_enabled: Optional[bool] = None
    is_driving_enabled: Optional[bool] = None

class Background(BaseModel):
    """
    가족 배경화면 앨범에 새로운 사진을 추가하기 위해 Client가 보내는 데이터
    """
    family_id: Optional[str] = None
    image_url: Optional[str] = None
