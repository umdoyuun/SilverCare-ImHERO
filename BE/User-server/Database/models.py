"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Table Models
"""

# Libraries
from sqlalchemy import Column, Date, TIMESTAMP, INT, FLOAT, Enum, TEXT, String, Boolean, ForeignKey, func
from sqlalchemy.orm import relationship
from sqlalchemy.ext.declarative import declarative_base

from enum import Enum as BaseEnum

# Create table base
Base = declarative_base()

# Enum
class Role(BaseEnum):
    TEST = "test"
    SYSTEM = "system"
    MAIN = "main"
    SUB = "sub"

class Gender(BaseEnum):
    MALE = "male"
    FEMALE = "female"
    OTHER = "other"

class Order(BaseEnum):
    ASC = "asc"
    DESC = "desc"

class NotificationGrade(BaseEnum):
    INFO = "info"
    WARN = "warn"
    CRIT = "crit"
    NONE = "none"

class Uploader(BaseEnum):
    ALL = "all"
    MINE = "mine"

# ========== DB Tables ==========

class AccountsTable(Base):
    """
    사용자 계정 정보
    """
    __tablename__ = "accounts"

    id = Column(String(16), primary_key=True, nullable=False)
    email = Column(String(128), nullable=False, unique=True)
    password = Column(String(128), nullable=False)
    role = Column(Enum(Role), nullable=False)
    user_name = Column(String(32), nullable=True)
    birth_date = Column(Date, nullable=True)
    gender = Column(Enum(Gender), nullable=True)
    address = Column(String(128), nullable=True)

    family_relations = relationship("FamiliesTable", cascade="all, delete")
    member_relations = relationship("MemberRelationsTable", cascade="all, delete")
    login_sessions = relationship("LoginSessionsTable", cascade="all, delete")
    message_sent = relationship("MessageTable", foreign_keys="[MessageTable.from_id]" ,cascade="all, delete")
    message_received = relationship("MessageTable", foreign_keys="[MessageTable.to_id]" ,cascade="all, delete")
    background_accounts_relations = relationship("BackgroundsTable", cascade="all, delete")

    def __repr__(self):
        return (f"" +
                f"<Account(id='{self.id}', " +
                f"email='{self.email}', " +
                f"password='{self.password}', " +
                f"role='{self.role}', " +
                f"user_name='{self.user_name}', " +
                f"birth_date='{self.birth_date}', " +
                f"gender='{self.gender}', " +
                f"address='{self.address}')>"
                )

class FamiliesTable(Base):
    """
    가족 정보
    """
    __tablename__ = "families"

    id = Column(String(16), primary_key=True, nullable=False)
    main_user = Column(String(16), ForeignKey('accounts.id'), nullable=False)
    family_name = Column(String(128), nullable=True)

    member_relations = relationship("MemberRelationsTable", cascade="all, delete")
    home_status = relationship("HomeStatusTable", cascade="all, delete")
    health_status = relationship("HealthStatusTable", cascade="all, delete")
    active_status = relationship("ActiveStatusTable", cascade="all, delete")
    mental_status = relationship("MentalStatusTable", cascade="all, delete")
    mental_reports = relationship("MentalReportsTable", cascade="all, delete")
    notifications_relations = relationship("NotificationsTable", cascade="all, delete")
    settings_relations = relationship("SettingsTable", cascade="all, delete")
    background_families_relations = relationship("BackgroundsTable", cascade="all, delete")

    def __repr__(self):
        return (f"" +
                f"<Family(id='{self.id}', " +
                f"main_user='{self.main_user}', " +
                f"family_name='{self.family_name}')>"
                )

class MemberRelationsTable(Base):
    """
    가족에 연관된 멤버 정보
    """
    __tablename__ = "memberrelations"

    id = Column(String(16), primary_key=True, nullable=False)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    user_id = Column(String(16), ForeignKey('accounts.id'), nullable=False)
    nickname = Column(String(32), nullable=True)

    def __repr__(self):
        return (f"" +
                f"<MemberRelation(id='{self.id}', " +
                f"family_id='{self.family_id}', " +
                f"user_id='{self.user_id}', " +
                f"nickname='{self.nickname}')>"
                )

class LoginSessionsTable(Base):
    """
    사용자의 로그인 세션 정보
    """
    __tablename__ = "loginsessions"

    xid = Column(String(32), primary_key=True, nullable=False)
    user_id = Column(String(16), ForeignKey('accounts.id'), nullable=False)
    last_active = Column(TIMESTAMP, nullable=True, server_default=func.now(), server_onupdate=func.now())
    is_main_user = Column(Boolean, nullable=False, server_default="FALSE")
    is_remember = Column(Boolean, nullable=False, server_default="FALSE")

    def __repr__(self):
        return (f"" +
                f"<LoginSession(xid='{self.xid}', " +
                f"user_id='{self.user_id}', " +
                f"last_active='{self.last_active}', " +
                f"is_main_user='{self.is_main_user}, " +
                f"is_remember='{self.is_remember}')>"
                )

class HomeStatusTable(Base):
    """
    집안 환경 정보
    """
    __tablename__ = "homestatus"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    reported_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    temperature = Column(FLOAT, nullable=True)
    humidity = Column(FLOAT, nullable=True)
    dust_level = Column(FLOAT, nullable=True)
    ethanol = Column(FLOAT, nullable=True)
    others = Column(TEXT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<HomeStatus(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"reported_at='{self.reported_at}', " +
                f"temperature='{self.temperature}', " +
                f"humidity='{self.humidity}', " +
                f"dust_level='{self.dust_level}', " +
                f"ethanol='{self.ethanol}', " +
                f"others='{self.others}')>"
        )

class HealthStatusTable(Base):
    """
    건강 정보
    """
    __tablename__ = "healthstatus"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    reported_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    heart_rate = Column(INT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<HealthStatus(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"reported_at='{self.reported_at}', " +
                f"heart_rate='{self.heart_rate}')>"
        )

class ActiveStatusTable(Base):
    """
    활동 정보
    """
    __tablename__ = "activestatus"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    reported_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    score = Column(INT, nullable=True)
    action = Column(String(32), nullable=True)
    is_critical = Column(Boolean, nullable=True)
    description = Column(TEXT, nullable=True)
    image_url = Column(TEXT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<ActiveStatus(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"reported_at='{self.reported_at}', " +
                f"score='{self.score}', " +
                f"action='{self.action}', " +
                f"is_critical='{self.is_critical}', " +
                f"description='{self.description}', " +
                f"image_url='{self.image_url}')>"
        )

class MentalStatusTable(Base):
    """
    단일 정신 건강 정보
    """
    __tablename__ = "mentalstatus"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    reported_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    score = Column(INT, nullable=True)
    is_critical = Column(Boolean, nullable=True)
    description = Column(TEXT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<MentalStatus(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"reported_at='{self.reported_at}', " +
                f"score='{self.score}', " +
                f"is_critical='{self.is_critical}', " +
                f"description='{self.description}')>"
        )

class MentalReportsTable(Base):
    """
    장기 정신 건강 리포트
    """
    __tablename__ = "mentalreports"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    reported_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    start_time = Column(TIMESTAMP, nullable=True)
    end_time = Column(TIMESTAMP, nullable=True)
    average_score = Column(INT, nullable=True)
    critical_days = Column(INT, nullable=True)
    best_day = Column(Date, nullable=True)
    worst_day = Column(Date, nullable=True)
    improvement_needed = Column(Boolean, nullable=True)
    summary = Column(TEXT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<MentalReport(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"reported_at='{self.reported_at}', " +
                f"start_time='{self.start_time}', " +
                f"end_time='{self.end_time}', " +
                f"average_score='{self.average_score}', " +
                f"critical_days='{self.critical_days}', " +
                f"best_day='{self.best_day}', " +
                f"worst_day='{self.worst_day}', " +
                f"improvement_needed='{self.improvement_needed}', " +
                f"summary='{self.summary}')>"
        )

class MasterRegionsTable(Base):
    """
    광역자치단체 정보
    """
    __tablename__ = "masterregions"

    region_name = Column(String(32), primary_key=True, nullable=False)
    region_type = Column(String(32), nullable=True)

    master_relation = relationship("SubRegionsTable", cascade="all, delete")

    def __repr__(self):
        return (f"" +
                f"<MasterRegion(region_name='{self.region_name}', " +
                f"region_type='{self.region_type}')>"
        )

class SubRegionsTable(Base):
    """
    기초자치단체 정보
    """
    __tablename__ = "subregions"

    main_region = Column(String(32), ForeignKey('masterregions.region_name'), primary_key=True, nullable=False)
    sub_region_name = Column(String(32), primary_key=True, nullable=False)
    region_type = Column(String(32), nullable=True)

    def __repr__(self):
        return (f"" +
                f"<SubRegion(main_region='{self.main_region}', " +
                f"sub_region='{self.sub_region_name}', " +
                f"region_type='{self.region_type}')>"
        )

class NotificationsTable(Base):
    """
    알림 및 재난, 긴급 정보
    """
    __tablename__ = "notifications"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    created_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    notification_grade = Column(Enum(NotificationGrade), nullable=True)
    descriptions = Column(TEXT, nullable=True)
    message_sn = Column(INT, nullable=True)
    is_read = Column(Boolean, nullable=True, server_default="FALSE")
    image_url = Column(TEXT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<Notification(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"created_at='{self.created_at}', " +
                f"notification_grade='{self.notification_grade}', " +
                f"descriptions='{self.descriptions}', " +
                f"message_sn='{self.message_sn}', " +
                f"is_read='{self.is_read}," +
                f"image_url='{self.image_url}')>"
        )

class MessageTable(Base):
    """
    사용자간의 주고 받는 메시지 정보
    """
    __tablename__ = "messages"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    from_id = Column(String(16), ForeignKey('accounts.id'), nullable=False)
    to_id = Column(String(16), ForeignKey('accounts.id'), nullable=False)
    created_at = Column(TIMESTAMP, nullable=True, server_default=func.now())
    content = Column(TEXT, nullable=True)
    image_url = Column(TEXT, nullable=True)
    is_read = Column(Boolean, nullable=True, server_default="FALSE")

    def __repr__(self):
        return (f"" +
                f"<Message(index='{self.index}', " +
                f"from_id='{self.from_id}', " +
                f"to_id='{self.to_id}', " +
                f"created_at='{self.created_at}', " +
                f"content='{self.content}', " +
                f"image_url='{self.image_url}', " +
                f"is_read='{self.is_read}')>"
        )

class NewsTable(Base):
    """
    한국의 뉴스 정보
    """
    __tablename__ = "news"

    id = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    title = Column(String(255), nullable=False)
    link = Column(TEXT, nullable=False)
    pub_date = Column(Date, nullable=False)
    image_url = Column(TEXT, nullable=True)
    category = Column(String(50), nullable=False)
    created_at = Column(TIMESTAMP, nullable=True, server_default=func.now())

    def __repr__(self):
        return (f"" +
                f"<News(id='{self.id}', " +
                f"title='{self.title}', " +
                f"link='{self.link}', " +
                f"pub_date='{self.pub_date}', " +
                f"image_url='{self.image_url}', " +
                f"category='{self.category}', " +
                f"created_at='{self.created_at}')>"
        )

class SettingsTable(Base):
    """
    로봇의 설정 값 정보
    """
    __tablename__ = "settings"

    family_id = Column(String(16), ForeignKey('families.id'), primary_key=True, nullable=False)
    is_alarm_enabled = Column(Boolean, nullable=False, server_default="FALSE")
    is_camera_enabled = Column(Boolean, nullable=False, server_default="FALSE")
    is_microphone_enabled = Column(Boolean, nullable=False, server_default="FALSE")
    is_driving_enabled = Column(Boolean, nullable=False, server_default="FALSE")

    def __repr__(self):
        return (f"" +
                f"<Settings(family_id='{self.family_id}', " +
                f"is_alarm_enabled='{self.is_alarm_enabled}', " +
                f"is_camera_enabled='{self.is_camera_enabled}', " +
                f"is_microphone_enabled='{self.is_microphone_enabled}', " +
                f"is_driving_enabled='{self.is_driving_enabled}')>"
        )

class BackgroundsTable(Base):
    """
    저장된 배경화면 정보
    """
    __tablename__ = "backgrounds"

    index = Column(INT, primary_key=True, nullable=False, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'), nullable=False)
    uploader_id = Column(String(16), ForeignKey('accounts.id'), nullable=False)
    image_url = Column(TEXT, nullable=True)

    def __repr__(self):
        return (f"" +
                f"<Background(index='{self.index}', " +
                f"family_id='{self.family_id}', " +
                f"uploader_id='{self.uploader_id}', " +
                f"image_url='{self.image_url}')>"
        )
