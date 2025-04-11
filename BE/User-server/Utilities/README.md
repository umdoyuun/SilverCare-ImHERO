## Carebot Project - Utilities

### 개요

Carebot Project는 독거노인을 위한 스마트 생활 도우미 서비스입니다. 이 서비스는 단순한 대화 상대를 넘어, 일상 속에서 동반자가 되어주고, 긴급한 상황에서는 신속한 도움을 제공하는 역할을 합니다.

또한, Carebot은 가족 및 요양보호사가 독거노인의 생활 상태를 원격으로 모니터링할 수 있도록 지원합니다. 대화 내역을 바탕으로 감정 및 심리 상태를 분석하고, 환경 및 활동 데이터를 수집하여 위험 요소를 감지함으로써 보다 안전한 생활을 돕습니다.

API에서는 반복적으로 사용되면서 일관된 결과 값을 요구되는 중요한 기능들이 존재합니다. 사용자 계정을 생성하기 위해 ID를 생성하고, 로그인을 위해 비밀번호를 검증하는 보안의 관련된 기능이 대표적입니다.

**반복적으로 사용되며 일관된 기능을 제공되길 요구**되는 기능을 별도의 도구로 정의하여 재사용할 수 있는 도구 모음이 Utilities 입니다.

### 기술

<img src="/uploads/cca32fa6cf2f85bb54b812c60262d92f/Python-Dark.svg" width="100" height="100" alt="Python"/>

| **분야** | **사용한 기술** |
| --- | --- |
| Program Language | **Python** 3.11.9 |
| Encryption | **bcrypt** 4.2.1 |

### 시스템 구조

1. **`auth_tools.py`**
    
    **랜덤한 ID나 Hex ID를 생성하고, 비밀번호를 검증**하는 기능을 제공합니다.
    
    ID를 생성하기 위해 UUID로 생성하는 방안을 고려했으나, ID는 URL에 입력되거나 사용자가 보고 수동으로 입력해야 하는 경우가 있기 때문에 너무 긴 ID를 생성하는 것은 좋지 않다고 판단하였습니다.
    
    그래서 별도로 **16자리의 숫자, 영문 소문자, 영문 대문자로 이뤄진 ID를 생성**할 수 있는 별도의 기능을 구현하였습니다.
    
    ```python
    def random_id(length: int = 16, set_type: Identify = Identify.USER):
        characters = string.ascii_letters + string.digits
        start = set_type.value[0].upper()
        new_id: str = start + ''.join(random.choice(characters) for _ in range(length - 1))
        return new_id
    ```
    
    ID의 첫 번째 자리는 어떤 종류의 ID인지 식별(`U` : User, `F` : Family, `M` : Member)하는 용도로 사용하며, 그 뒤는 랜덤한 숫자, 영문 소문자, 영문 대문자로 이뤄져 있습니다.
    
    또한, 비밀번호를 Database에 암호화 하여 저장하기 위해 **bcrypt**를 사용하고 있습니다.
    
    ```python
    def hash_password(plain_password: str) -> str:
        salt = bcrypt.gensalt()
        hashed_password = bcrypt.hashpw(plain_password.encode('utf-8'), salt)
        decoded_password = hashed_password.decode('utf-8')
        return decoded_password
    
    ```
    
2. **`check_tools.py`**
    
    **유효한 데이터를 입력했는지 점검**하는 기능을 제공합니다.
    
    Client에서 보낸 **date(날짜)정보**가 존재하는 날짜인지 확인하는 기능입니다. 복잡한 기능으로 구현하지 않고, python의 `date()` 생성을 활용하여 **`ValueError`**가 발생하는 경우 유효하지 않다고 판단하도록 설계하였습니다.
    
    ```python
    def is_valid_date(raw_date: Date) -> bool:
        try:
            date(year=raw_date.year, month=raw_date.month, day=raw_date.day)
            return True
        except ValueError:
            return False
    
    ```
    
3. **`logging_tools.py`**
    
    API 서버의 **Log Message를 지정된 형식으로 출력**되는 기능을 제공합니다.
    
    처리되는 내용이 많은 만큼 발생될 오류가 많기 때문에 형식화된 Log기능이 필요하게 됩니다. 이를 위해 python에 내장된 logging의 Config를 수정하여 통일된 형식으로 출력되게 만들었습니다.
    
    ```python
    import logging
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname).4s:     [%(name)s] %(message)s",
    )
    ```
    
    또한, **`logger`**를 제공하여 **다른 부분에서도 Log를 쉽게 출력할 수 있도록 제공**하고 있습니다.
    

### 기능 정의

> **auth_tools 부분**
> 

| Order | Function Name  | Description | Return |
| --- | --- | --- | --- |
| 1 | `random_id(length, set_type)` | 랜덤한 ID를 생성하는 기능 | `str` |
| 2 | `random_xid(length_byte)` | 램덤한 Hex ID를 생성하는 기능 | `str` |
| 3 | `hash_password(plain_password)` | 암호화된 비밀번호로 변경하는 기능 | `str` |
| 4 | `verify_password(input_password, hashed_password)` | 입력한 비밀번호와 DB의 비밀번호가 일치하는지 검증하는 기능 | `bool` |

> **check_tools 부분**
> 

| Order | Function Name  | Description | Return |
| --- | --- | --- | --- |
| 1 | `is_valid_date(raw_date)` | 날짜 정보가 유효한지 확인하는 기능 | `bool` |

> **logging_tools 부분**
> 

| Order | Function Name  | Description | Return |
| --- | --- | --- | --- |
| 1 | `get_logger(name)` | 해당 Level에 맞는 logger 기능을 불러오는 기능 | `Logger` |