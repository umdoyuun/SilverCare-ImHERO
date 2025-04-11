"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot User API Server ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛
Server Authentication Tools
"""

# Libraries
import random
import string
import bcrypt
from enum import Enum

from secrets import token_hex

class Identify(Enum):
    USER = "user"
    FAMILY = "family"
    MEMBER = "member"


def random_id(length: int = 16, set_type: Identify = Identify.USER):
    """
    각종 ID를 난수로 생성하는 기능
    :param length: ID의 길이
    :param set_type: 생성하려는 ID의 종류
    :return: 문자와 숫자가 혼합된 length 길이의 ID
    """

    characters = string.ascii_letters + string.digits
    start = set_type.value[0].upper()
    new_id: str = start + ''.join(random.choice(characters) for _ in range(length - 1))
    return new_id

def random_xid(length_byte: int = 16) -> str:
    """
    각종 16진수 xid를 난수로 생성하는 기능
    :param length_byte: ID의 Byte 길이 (16 -> 32자리의 16진수 string)
    :return: 16진수로 이뤄진 length_byte 길이의 xid
    """
    new_xid: str = token_hex(length_byte)
    return new_xid

def hash_password(plain_password: str) -> str:
    """
    평문 비밀번호를 암호화된 비밀번호로 변경해주는 기능
    :param plain_password: 평문 비밀번호
    :return: 암호화된 비밀번호
    """

    salt = bcrypt.gensalt()
    hashed_password = bcrypt.hashpw(plain_password.encode('utf-8'), salt)
    decoded_password = hashed_password.decode('utf-8')
    return decoded_password


def verify_password(input_password: str, hashed_password: str) -> bool:
    """
    사용자가 입력한 평문 비밀번호와 저장된 암호화 비밀번호가 일치하는지 확인하는 기능
    :param input_password: 사용자가 입력한 평문 비밀번호
    :param hashed_password: DB에 저장된 암호화 비밀번호
    :return: True -> 일치함, False -> 불일치함
    """
    result: bool = bcrypt.checkpw(input_password.encode('utf-8'), hashed_password.encode('utf-8'))
    return result

