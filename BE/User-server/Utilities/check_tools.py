"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Server Authentication Tools
"""

# Libraries
from datetime import date
from Endpoint.models import *

def is_valid_date(raw_date: Date) -> bool:
    """
    입력된 연월일이 유효한지 확인
    :param raw_date: 날짜 데이터 Date
    :return: 유효한지 여부 bool
    """
    try:
        date(year=raw_date.year, month=raw_date.month, day=raw_date.day)
        return True
    except ValueError:
        return False
