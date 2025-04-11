"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
Database Table Models
"""

# Library
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
    is_main_user = Column(Boolean, nullable=False)

    def __repr__(self):
        return (f"" +
                f"<LoginSession(xid='{self.xid}', " +
                f"user_id='{self.user_id}', " +
                f"last_active='{self.last_active}', " +
                f"is_main_user='{self.is_main_user}')>"
                )
