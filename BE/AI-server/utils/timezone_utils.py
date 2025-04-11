from datetime import datetime, timezone, timedelta

KST = timezone(timedelta(hours=9))

def get_kst_now() -> datetime:
    """현재 시간을 KST로 반환"""
    return datetime.now(timezone.utc).astimezone(KST)

def get_kst_today():
    """현재 날짜를 KST로 반환"""
    return get_kst_now().date()

def to_kst(dt: datetime) -> datetime:
    """주어진 datetime을 KST로 변환"""
    if dt.tzinfo is None:  # naive datetime인 경우 UTC로 가정
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(KST)

def to_utc(dt: datetime) -> datetime:
    """주어진 datetime을 UTC로 변환 """
    if dt.tzinfo is None:  # naive datetime인 경우 KST로 가정
        dt = dt.replace(tzinfo=KST)
    return dt.astimezone(timezone.utc)

def to_utc_start_of_day(date_val) -> datetime:
    """주어진 날짜의 시작(00:00:00 KST)을 UTC datetime으로 변환"""
    kst_datetime = datetime.combine(date_val, datetime.min.time(), tzinfo=KST)
    return kst_datetime.astimezone(timezone.utc)

def to_utc_end_of_day(date_val) -> datetime:
    """주어진 날짜의 끝(23:59:59.999999 KST)을 UTC datetime으로 변환"""
    kst_datetime = datetime.combine(date_val, datetime.max.time(), tzinfo=KST)
    return kst_datetime.astimezone(timezone.utc)