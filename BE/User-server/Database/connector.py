"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Connector
"""

# Libraries
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

import os
from dotenv import load_dotenv

class Database:
    def __init__(self):
        load_dotenv()  # database environment 불러오기

        self.host = os.getenv("DB_HOST")
        self.port = int(os.getenv("DB_PORT", 3306))
        self.user = os.getenv("DB_USER")
        self.password = os.getenv("DB_PASSWORD")
        self.schema = os.getenv("DB_SCHEMA")
        self.charset = os.getenv("DB_CHARSET", "utf8")

        # Connection Pool 방식 SQL 연결 생성
        self.engine = create_engine(
            f"mysql+pymysql://{self.user}:"+
            f"{self.password}@{self.host}:{self.port}/"+
            f"{self.schema}?charset={self.charset}",
            pool_size=10,
            max_overflow=5,
            pool_recycle=120,
            pool_pre_ping=True,
            echo=False
        )

        # ORM Session 설정
        self.pre_session = sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=self.engine
        )

    # DB 연결을 위한 Pre Session을 반환하는 기능
    def get_pre_session(self):
        return self.pre_session

database_instance = Database()
