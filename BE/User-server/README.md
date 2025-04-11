## Carebot Project - User API

### 개요

Carebot Project는 독거노인을 위한 스마트 생활 도우미 서비스입니다. 이 서비스는 단순한 대화 상대를 넘어, 일상 속에서 동반자가 되어주고, 긴급한 상황에서는 신속한 도움을 제공하는 역할을 합니다.

또한, Carebot은 가족 및 요양보호사가 독거노인의 생활 상태를 원격으로 모니터링할 수 있도록 지원합니다. 대화 내역을 바탕으로 감정 및 심리 상태를 분석하고, 환경 및 활동 데이터를 수집하여 위험 요소를 감지함으로써 보다 안전한 생활을 돕습니다.

User API Server는 사용자 정보와 가족 관리, 상태 정보 보고 및 조회, AI Chat 및 심리 상태 보고서 생성,  날씨와 뉴스 등의 부가 서비스 제공합니다. 즉, 사용자가 접근하는 **Web Platform과 Carebot 사이의 연결 다리 역할**을 수행하게 됩니다.

감정 및 심리 상태 보고서, 건강 정보와 같은 민감한 정보를 다루기 때문에 모든 서비스는 권한이 부여된 사용자만 접근할 수 있도록 Session 기반 인증으로 설계되었으며, 통신 방식도 HTTPS 프로토콜을 사용하여 Packet의 Payload에도 노출되지 않도록 하였습니다.

### 기술

<img src="/uploads/cca32fa6cf2f85bb54b812c60262d92f/Python-Dark.svg" width="100" height="100" alt="Python"/>
<img src="/uploads/3e55daa5ca0afaebcaec2f9777f4bb23/FastAPI.svg" width="100" height="100" alt="FastAPI"/>
<img src="/uploads/9e181b0d15199ae7d5b8ad673644b30b/MySQL-Dark.svg" width="100" height="100" alt="MySQL"/>

| **분야** | **사용한 기술** |
| --- | --- |
| Program Language | **Python** 3.11.9 |
| Server Architecture | **FastAPI** 0.115.6 |
| Database | **MariaDB** 10.3.23 |
| DB Library | **SQL-Alchemy** 2.0.37 |
| Validation | **Pydantic** 2.10.5 |
| Connector | **HTTPX** 0.28.1 |
| **Encryption** | **bcrypt** 4.2.1 |

### 프로젝트 구조

![최종서비스](/uploads/c224e30672147ae64917ff9bf1bd0ab6/최종서비스.png)

이 Repository는 Backend Server 구조 중에서 **API Server** 부분에 해당합니다.

```
│  docker-compose-dev.yml
│  Dockerfile
│  main.py
│  README.md
│  requirements.txt
│
├─Database
│  │  accounts.py
│  │  authentication.py
│  │  connector.py
│  │  families.py
│  │  members.py
│  │  messages.py
│  │  models.py
│  │  notifications.py
│  │  README.md
│  │  status.py
│  │  tools.py
│  └─ __init__.py
│
├─Endpoint
│  │  models.py
│  └─ README.md
│
├─External
│  │  ai.py
│  └─ README.md
│
├─Routers
│  │  accounts.py
│  │  authentication.py
│  │  chats.py
│  │  families.py
│  │  members.py
│  │  messages.py
│  │  notifications.py
│  │  README.md
│  │  status.py
│  └─ tools.py
│
└─Utilities
   │  auth_tools.py
   │  check_tools.py
   │  logging_tools.py
   └─ README.md

```

### 목적 및 기능

1. **`docker-compose-dev.yml`**
    
    배포를 위해서 필요한 **Docker Container의 설정 정보가 포함**된 Docker Compose 정보입니다.
    
2. **`Dockerfile`**
    
    배포할 이미지에 대한 **기본 이미지와 파일들에 대한 정의가 포함**된 Docker 정보입니다.
    
3. **`main.py`**
    
    User API Server를 구성하는 **FastAPI와 CORS 설정, Router 설정**이 포함되어 있습니다.
    
4. **`requirements.txt`**
    
    해당 서비스를 수행하기 위해 설치해야 하는 Python Library의 종류와 버전이 기록된 문서입니다.
    
    아래는 필수적으로 설치해야 하는 Library의 목록과 기능, 버전에 대한 설명입니다.
    
    | Library | Description | Version |
    | --- | --- | --- |
    | **`fastapi`** | API & Web Framework | `0.115.6` |
    | **`uvicorn`** | ASGI web server | `0.34.0` |
    | **`PyMySQL`** | MySQL Library | `1.1.1` |
    | **`SQLAlchemy`** | SQL Toolkit and ORM | `2.0.37` |
    | **`python-detenv`** | Python Environment Library | `1.0.1` |
    | **`pydantic`** | Data Validation Library | `2.10.5` |
    | **`httpx`** | Python HTTP Client | `0.28.1` |
5. **`Database`**
    
    Database와 연결하여 **정보를 생성하고 수정, 삭제하는 기능**이 포함된  Library입니다.
    
6. **`Endpoint`**
    
    Client에서 보내는 요청에 담긴 **Body 내용을 사전에 정의한 객체**가 포함되어 있습니다.
    
7. **`External`**
    
    AI Process Server와 연계하여 **AI 기반의 서비스**를 제공하는 기능이 포함되어 있습니다.
    
8. **`Routers`**
    
    Client에서 사용될 **모든 API의 기능**이 포함되어 있습니다.
    
9. **`Utilities`**
    
    Database나 Routers에 **사용되는 도구**들이 정의되어 있습니다.

### 변경 기록

- **[Release] `0.1.0`**
    - 기본적인 Database Connector 기능 완성
- **[Release] `0.2.0`**
    - Accounts 기능 완성
- **[Add] `0.2.1`**
    - 배포를 위한 Docker 및 Docker compose 생성
    - Develop mode와 Release mode 구별 기능 완성
- **[Fix]** **`0.2.2`**
    - 환경 변수가 적용되지 않는 문제 해결
    - DB Session이 계속 만료되는 문제 해결
- **[Add] `0.2.3`**
    - Families 기능 완성
    - Database 기능 코드 분리 완료
- **[Add] `0.2.4`**
    - Members 기능 완성
- **[Release] `0.3.0`**
    - Authentication 기능 완성
    - 사용자 정보를 이용한 검증 기능 추가
- **[Fix] `0.3.1`**
    - 사용자 정보 검증을 위한 Cookie 정책을 현재 서비스에 맞게 수정
    - 사용자 정보 검증을 위한 Cookie 정책을 현재 서비스에 맞게 수정
- **[Fix] `0.3.2`**
    - 사용자의 활동이 있을 때 만료 시간이 연장되지 않는 문제 해결
    - Accounts 접근에 권한 검증 추가
- **[Release] `0.4.0`**
    - Status 기능 완성
- **[Add] `0.4.1`**
    - AI Process 연동 기능 완성
- **[Fix] `0.4.2`**
    - 서버 시간을 사용하지 않아 발생하는 문제를 해결
    - AI Process의 처리 시간을 기다리기 위해 Timeout 시간 연장
- **[Fix] `0.4.3`**
    - 회원 가입 처리 시 주소가 같이 저장되지 않는 문제 해결
- **[Add] `0.4.4`**
    - 대한민국의 광역과 기초 자치단체 데이터를 받아올 수 있는 기능 추가
    - Client에서 받은 날짜 정보를 검증하는 방식 개선
- **[Release] `0.5.0`**
    - 주 사용자의 정보를 이용해 가족을 찾는 기능 추가
- **[Fix] `0.5.1`**
    - Log 기능을 `Print()` 대신 `Logger()`로 변경
- **[Release] `0.6.0`**
    - Notifications 기능 완성
- **[Add] `0.6.1`**
    - AI Process의 News, Weather 기능 연동
- **[Release] `0.7.0`**
    - Messages 기능 완성
- **[Add] `0.7.1`**
    - Notifications를 불러오는 부분에 시간으로 필터링하는 기능 추가
    - Database Connection overflow를 방지하기 위한 기능 추가
- **[Fix] `0.7.2`**
    - Message의 Accounts Table 외래키 관계 문제 해결
    - Database Connector가 여러 군데에서 호출되는 문제 해결
- **[Release] `0.8.0`**
    - Active Status에 Image를 추가할 수 있도록 변경
    - Message를 불러올 때 없으면 404 Not Found 대신 200 OK + 빈 리스트를 보내도록 변경
- **[Add] `0.8.1`**
    - 주 사용자가 가족에 등록된 Member를 추방할 수 있는 기능 추가
    - Notifications에 이미지를 포함할 수 있는 기능 추가
- **[Release] `0.9.0`**
    - 자동 로그인 및 Session 검증하는 기능 완성
- **[Add] `0.9.1`**
    - Family, Member 부분 권한 검증 추가
    - Status의 반환 값을 “data” → “result”가 되도록 변경
- **[Add] `0.9.2`**
    - News를 Cache된 News를 불러오도록 수정
    - Carebot의 Settings를 불러오고 변경할 수 있는 기능 완성
- **[Release] `1.0.0`**
    - 최종 버전 배포
- **[Fix] `1.0.1`**
    - Family를 생성하기 전에 Settings를 생성하려는 문제 해결
- **[Add] `1.0.2`**
    - Family를 등록하기 위해 ID 검증과 이름을 확인하는 기능 추가
    - 가족 단위 별 Background를 추가, 삭제, 불러오는 기능 추가
- **[Add] `1.0.3`**
    - Main User의 활동이 3일 이상  없는 경우에 Session을 삭제하도록 기능 추가
- **[Add] `1.0.4`**
    - Session 권한 점검할 때 Cookie 점검 대신 Body에 Session ID를 담게 하고 브라우저에 Cookie가 저장되도록 변경
    - 여러 개의 메시지와 알림을 읽음 처리할 수 있는 기능 추가