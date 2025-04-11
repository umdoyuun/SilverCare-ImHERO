"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Server Authentication Tools
"""

# Library
from enum import Enum


class Identify(Enum):
    USER = "user"
    FAMILY = "family"
    MEMBER = "member"
