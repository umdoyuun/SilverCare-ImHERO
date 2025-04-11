import os
import logging
from sqlalchemy import create_engine, event, Column, String, DateTime, Integer, Text, ForeignKey, Date, Enum, Float, Boolean, func
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship, sessionmaker, Session
from sqlalchemy.pool import Pool
from sqlalchemy.exc import OperationalError
from dotenv import load_dotenv
from time import sleep
from datetime import datetime, timezone, date

# 환경 변수 로드
load_dotenv()

# 로깅 설정
logger = logging.getLogger(__name__)

Base = declarative_base()

# 모델 정의
class ChatSession(Base):
    __tablename__ = 'chatsessions'
    uid = Column('uid', String(36), primary_key=True)
    user_id = Column('user_id', String(16))
    created_at = Column(DateTime(timezone=True), server_default=func.now()) 
    last_active = Column('last_active', DateTime(timezone=True))  
    conversations = relationship("ChatHistory", back_populates="session")

class ChatHistory(Base):
    __tablename__ = 'chathistories'
    index = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String(50), ForeignKey('chatsessions.uid'))
    user_id = Column(String(16))
    user_message = Column(Text)
    bot_message = Column(Text)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    session = relationship("ChatSession", back_populates="conversations")

class LocationMap(Base):
    __tablename__ = 'locationmaps'
    address = Column(String(128), primary_key=True)
    x_value = Column(Integer)
    y_value = Column(Integer)

class MentalStatus(Base):
    __tablename__ = 'mentalstatus'
    index = Column(Integer, primary_key=True, autoincrement=True)
    family_id = Column(String(16))
    reported_at = Column(DateTime(timezone=True), server_default=func.now())
    score = Column(Integer)
    is_critical = Column(Integer)
    description = Column(Text)

class Account(Base):
    __tablename__ = 'accounts'
    id = Column(String(16), primary_key=True)
    email = Column(String(128))
    password = Column(String(128))
    role = Column(Enum('TEST', 'SYSTEM', 'MAIN', 'SUB'), nullable=False)
    user_name = Column(String(32))
    birth_date = Column(Date)
    gender = Column(Enum('MALE', 'FEMALE', 'OTHER'))
    address = Column(String(128))

class FallDetection(Base):
    __tablename__ = 'falldetections'
    index = Column(Integer, primary_key=True, autoincrement=True)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    image_path = Column(String(255), nullable=True)
    user_id = Column(String(16), ForeignKey('accounts.id'))

class Notification(Base):
    __tablename__ = 'notifications'
    index = Column(Integer, primary_key=True, autoincrement=True)
    family_id = Column(String(16))
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    notification_grade = Column(Enum('INFO', 'WARN', 'CRIT', 'NONE', name='notification_grade'))
    descriptions = Column(Text)
    message_sn = Column(Integer)

class Family(Base):
    __tablename__ = 'families'
    id = Column(String(16), primary_key=True)
    main_user = Column(String(16))
    family_name = Column(String(128))

class MemberRelations(Base):
    __tablename__ = 'memberrelations'
    id = Column(String(16), primary_key=True)
    family_id = Column(String(16))
    user_id = Column(String(16))
    nickname = Column(String(32))

class Message(Base):
    __tablename__ = 'messages'
    index = Column(Integer, primary_key=True, autoincrement=True)
    from_id = Column(String(16))
    to_id = Column(String(16))
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    content = Column(Text)
    image_url = Column(String(32))
    is_read = Column(Integer, default=0)

class ChatKeywords(Base):
    __tablename__ = 'chatkeywords'
    id = Column(Integer, primary_key=True, autoincrement=True)
    family_id = Column(String(16), ForeignKey('families.id'))
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    keywords = Column(Text)


class MentalReport(Base):
    __tablename__ = 'mentalreports'
    index = Column(Integer, primary_key=True, autoincrement=True)
    family_id = Column(String(16))
    reported_at = Column(DateTime(timezone=True), server_default=func.now())
    start_time = Column(DateTime(timezone=True))
    end_time = Column(DateTime(timezone=True))
    average_score = Column(Float)
    critical_days = Column(Integer)
    best_day = Column(Date)
    worst_day = Column(Date)
    improvement_needed = Column(Boolean)
    summary = Column(Text)

class News(Base):
    __tablename__ = 'news'
    id = Column(Integer, primary_key=True)
    title = Column(String(255), nullable=False)
    link = Column(Text, nullable=False)
    pub_date = Column(Date, nullable=False)
    image_url = Column(Text)
    category = Column(String(50), nullable=False)
    created_at = Column(DateTime(timezone=True))