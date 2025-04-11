"""
┏━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Care-bot Image Provider ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━┛
Server Logging Tools
"""

# Libraries
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s: \t  [%(name)s] %(message)s",
)


# 각 파일별 logger 반환 기능
def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)