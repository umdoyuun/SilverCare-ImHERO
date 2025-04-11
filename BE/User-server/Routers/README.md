## Carebot Project - Routers

### 개요

Carebot Project는 독거노인을 위한 스마트 생활 도우미 서비스입니다. 이 서비스는 단순한 대화 상대를 넘어, 일상 속에서 동반자가 되어주고, 긴급한 상황에서는 신속한 도움을 제공하는 역할을 합니다.

또한, Carebot은 가족 및 요양보호사가 독거노인의 생활 상태를 원격으로 모니터링할 수 있도록 지원합니다. 대화 내역을 바탕으로 감정 및 심리 상태를 분석하고, 환경 및 활동 데이터를 수집하여 위험 요소를 감지함으로써 보다 안전한 생활을 돕습니다.

Carebot과 User Platform에 정보와 기능을 제공하기 위해 API Server를 구축하는 것이 필수적입니다. Carebot 장비가 API Server에 직접 접근하여 생성된 알림과 문자를 수신하고, 위급상황을 빠르게 보고해야하는 기능이 요구되므로 빠른 처리와 응답이 중요하다고 생각했습니다.

또한, AI를 이용한 서비스와 개발 호환성을 위해 Python 기반의 Server가 필요했기에 부가 기능을 덜고 처리 속도에 집중한 **FastAPI**를 이용해서 API를 제작 하였습니다.

### 기술

<img src="/uploads/cca32fa6cf2f85bb54b812c60262d92f/Python-Dark.svg" width="100" height="100" alt="Python"/>
<img src="/uploads/3e55daa5ca0afaebcaec2f9777f4bb23/FastAPI.svg" width="100" height="100" alt="FastAPI"/>

| **분야** | **사용한 기술** |
| --- | --- |
| Program Language | **Python** 3.11.9 |
| Server Architecture | **FastAPI** 0.115.6 |

### 시스템 구조

1. **`accounts.py`**
    
    회원 가입과 회원 정보 수정, 탈퇴 등의 **사용자 계정**과 관련된 기능이 포함되어 있습니다.
    
2. **`authentication.py`**
    
    로그인, 로그아웃, 비밀번호 변경, 권한 확인 등의 **권한**과 관련된 기능이 포함되어 있습니다.
    
3. **`families.py`**
    
    **가족**을 새롭게 구성하고, 가족을 조회 및 수정하는 기능이 포함되어 있습니다.
    
4. **`members.py`**
    
    **가족에 등록된 구성원**을 추가 및 삭제, 수정하는 기능이 포함되어 있습니다.
    
5. **`status.py`**
    
    집 안 환경 정보, 건강 정보, 활동 정보, 감정 상태와 감정 및 심리 보고서를 생성 등의 **상태**와 관련된 기능이 포함되어 있습니다.
    
6. **`chats.py`**
    
    **AI와 이야기를 주고 받을 수** 있는 기능이 포함되어 있습니다.
    
7. **`messages.py`**
    
    독거 노인과 보호자 간의 **사진과 메시지**를 주고 받을 수 있는 기능이 포함되어 있습니다.
    
8. **`notifications.py`**
    
    Carebot이 **현재 상태**에 대한 알림을 보내거나, 거주하고 있는 지역의 **재난 정보**를 알려주는 기능이 포함되어 있습니다.
    
9. **`tools.py`**
    
    부가적으로 필요한 **날씨**나 **뉴스 정보**를 얻고, Carebot의 **작동 상태**를 보고 및 확인하는 기능이 포함되어 있습니다.
    

### 기능 정의

> **`/accounts` - 사용자 계정 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/accounts/check-email` | 중복된 이메일인지 확인하기 | [바로가기](https://www.notion.so/17e0abcedc588052aa0ee951d593be14?pvs=21) |
| 2 | **`POST`** | `/accounts` | 새로운 사용자 계정 생성하기 | [바로가기](https://www.notion.so/1810abcedc588035a3cbe10dd924d34c?pvs=21) |
| 3 | **`GET`** | `/accounts` | 모든 사용자 계정 정보 가져오기 | [바로가기](https://www.notion.so/1850abcedc5880bc99fef5bc60a44dad?pvs=21) |
| 4 | **`GET`** | `/accounts/:user-id` | 사용자 계정 정보 가져오기 | [바로가기](https://www.notion.so/1850abcedc588021a238cf08ffaa8d9b?pvs=21) |
| 5 | **`PATCH`** | `/accounts/:user-id` | 사용자 계정 정보 변경하기 | [바로가기](https://www.notion.so/1850abcedc588021976af41691ebfe0f?pvs=21) |
| 6 | **`DEL`** | `/accounts/:user_id` | 사용자 계정 삭제하기 | [바로가기](https://www.notion.so/1850abcedc5880d0803bcd9a2039a65a?pvs=21) |

> **`/auth` - 권한 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/auth/login` | 로그인 | [바로가기](https://www.notion.so/18c0abcedc588084af6acaa5d6e38f61?pvs=21) |
| 2 | **`POST`** | `/auth/logout` | 로그아웃 | [바로가기](https://www.notion.so/18c0abcedc58802296cdc11a75d486d6?pvs=21) |
| 3 | **`PATCH`** | `/auth/change-password` | 사용자의 비밀번호 변경하기 | [바로가기](https://www.notion.so/18c0abcedc5880cb8107d474f0fa9614?pvs=21) |
| 4 | **`PATCH`** | `/auth/auto-login` | 자동 로그인 활성화하기 | [바로가기](https://www.notion.so/1990abcedc5880e0b80ffd8b6e7b50cf?pvs=21) |
| 5 | **`POST`** | `/auth/check` | 현재 세션 권한 점검하기 | [바로가기](https://www.notion.so/1990abcedc5880d2921df2917b8e0d39?pvs=21) |

> **`/families` - 가족 관리 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/families/check-exist` | 가족에 소속되어 있는지 확인하기 | [바로가기](https://www.notion.so/18c0abcedc58808eb1e9d1d3b06eb7d4?pvs=21) |
| 2 | **`POST`** | `/families` | 새로운 가족 생성하기 | [바로가기](https://www.notion.so/18c0abcedc58803392e0ea7794847623?pvs=21) |
| 3 | **`GET`** | `/families` | 모든 가족 정보 가져오기 | [바로가기](https://www.notion.so/18c0abcedc588038a151c23d600a51b3?pvs=21) |
| 4 | **`POST`** | `/families/find` | 개인 정보로 가족 찾기 | [바로가기](https://www.notion.so/1930abcedc588024b950ef230601f25d?pvs=21) |
| 5 | **`GET`** | `/families/:family-id` | 가족 정보 가져오기 | [바로가기](https://www.notion.so/18c0abcedc5880599a49f2c314e0dd8d?pvs=21) |
| 6 | **`GET`** | `/families/name/:family-id` | 가족 유효성 확인 및 이름 가져오기 | [바로가기](https://www.notion.so/19d0abcedc5880bb8613d4706f951f67?pvs=21) |
| 7 | **`PATCH`** | `/families/:family-id` | 가족 정보 수정하기 | [바로가기](https://www.notion.so/18c0abcedc5880afae8cf28733117282?pvs=21) |
| 8 | **`DEL`** | `/families/:family-id` | 가족 삭제하기 | [바로가기](https://www.notion.so/18c0abcedc588088b224d727e6314b7c?pvs=21) |

> **`/members` - 가족 구성원 관리 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/members` | 새로운 가족 구성원을 추가하기 | [바로가기](https://www.notion.so/18c0abcedc5880a98917c7be08f6caa8?pvs=21) |
| 2 | **`GET`** | `/members?familyId&userId` | 조건으로 가족 구성원 찾기 | [바로가기](https://www.notion.so/18c0abcedc5880d7acb7fe5575bb3cad?pvs=21) |
| 3 | **`GET`** | `/members/:member-id` | 가족 구성원 정보 가져오기 | [바로가기](https://www.notion.so/18c0abcedc58805da600d61671b9daca?pvs=21) |
| 4 | **`PATCH`** | `/members/:member-id` | 가족 구성원 정보 수정하기 | [바로가기](https://www.notion.so/18c0abcedc58809db4a6c3fb679e0285?pvs=21) |
| 5 | **`DEL`** | `/members/:member-id` | 가족에서 구성원 삭제하기 | [바로가기](https://www.notion.so/18c0abcedc58801488cdd2fc6a4b3224?pvs=21) |
| 6 | **`DEL`** | `/members/kick/:member-id` | 가족에서 구성원 내보내기 | [바로가기](https://www.notion.so/1990abcedc58802480f8d48714bfdbd1?pvs=21) |

> **`/status` - 상태 보고 및 조회 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/status/home` | 집 안 환경 정보 보고하기 | [바로가기](https://www.notion.so/1900abcedc58808e9b67e69e07486cfa?pvs=21) |
| 2 | **`GET`** | `/status/home/:family-id?start&end&order` | 집 안 환경 정보 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880cf9fa5f55ae67de594?pvs=21) |
| 3 | **`GET`** | `/status/home/latest/:family-id` | 최신 집 안 환경 정보 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880648f9ad055e3ac4982?pvs=21) |
| 4 | **`DEL`** | `/status/home/latest/:family-id` | 최신 집 안 환경 정보 삭제하기 | [바로가기](https://www.notion.so/1900abcedc5880238be6db243b9e8d09?pvs=21) |
| 5 | **`POST`** | `/status/health` | 건강 상태 정보 보고하기 | [바로가기](https://www.notion.so/1900abcedc5880c3a3e7ea39f85bdc5d?pvs=21) |
| 6 | **`GET`** | `/status/health/:family-id?start&end&order` | 건강 상태 정보 가져오기 | [바로가기](https://www.notion.so/1900abcedc588099b897e099d7e0e930?pvs=21) |
| 7 | **`GET`** | `/status/health/latest/:family-id` | 최신 건강 상태 정보 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880838d34f94538063ca8?pvs=21) |
| 8 | **`DEL`** | `/status/health/latest/:family-id` | 최신 건강 상태 정보 삭제하기 | [바로가기](https://www.notion.so/1900abcedc58802ab958c944d547bd65?pvs=21) |
| 9 | **`POST`** | `/status/active` | 활동 정보 보고하기 | [바로가기](https://www.notion.so/1900abcedc5880688e28c8bc5b39d814?pvs=21) |
| 10 | **`GET`** | `/status/active/:family-id?start&end&order` | 활동 정보 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880afa59aee10bd39a78f?pvs=21) |
| 11 | **`GET`** | `/status/active/latest/:family-id` | 최신 활동 정보 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880879614c8c17551e2aa?pvs=21) |
| 12 | **`DEL`** | `/status/active/latest/:family-id` | 최신 활동 정보 삭제하기 | [바로가기](https://www.notion.so/1900abcedc588048ab46f82e8614edf5?pvs=21) |
| 13 | **`GET`** | `/status/mental/new/:family-id` | 단일 감정 상태 보고서 생성하기 | [바로가기](https://www.notion.so/1900abcedc5880918f5cc19f2da0829e?pvs=21) |
| 14 | **`GET`** | `/status/mental/:family-id?start&end&order` | 단일 감정 상태 보고서 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880df8a43d8a7d4673fa7?pvs=21) |
| 15 | **`GET`** | `/status/mental/latest/:family-id` | 최신 단일 감정 상태 보고서 가져오기 | [바로가기](https://www.notion.so/1900abcedc5880df966bf11f492e68f0?pvs=21) |
| 16 | **`DEL`** | `/status/mental/latest/:family-id` | 최신 단일 감정 상태 보고서 삭제하가 | [바로가기](https://www.notion.so/1900abcedc588086ac23c2375285a4cf?pvs=21) |
| 17 | **`GET`** | `/status/mental-reports/new/:family-id?start&end` | 장기간 감정 상태 보고서 생성하기 | [바로가기](https://www.notion.so/1920abcedc5880f5a9f7c385af584bfc?pvs=21) |
| 18 | **`GET`** | `/status/mental-reports/:family-id?start&end&order` | 장기간 감정 상태 보고서 가져오기 | [바로가기](https://www.notion.so/1920abcedc5880248b59c3a61f8b938e?pvs=21) |
| 19 | **`GET`** | `/status/mental-reports/latest/:family-id` | 최신 장기간 감정 상태 보고서 가져오기 | [바로가기](https://www.notion.so/1920abcedc58803b9c54c5cf6fd9e940?pvs=21) |
| 20 | **`DEL`** | `/status/mental-reports/latest/:family-id` | 최신 장기간 감정 상태 보고서 삭제하기 | [바로가기](https://www.notion.so/1920abcedc5880a1ace4eb12e28dd8ee?pvs=21) |
| 21 | **`GET`** | `/status/keywords/:family-id` | 오늘의 대화 키워드 분석 및 생성하기 | [바로가기](https://www.notion.so/1990abcedc5880e3aa58dcaaad167e0a?pvs=21) |
| 22 | **`GET`** | `/status/psychology/:family-id?start&end` | 심리 분석 보고서 생성하기 | [바로가기](https://www.notion.so/1990abcedc5880a38be9c4eb9699c0f0?pvs=21) |

> **`/chats`, `/message` - AI 대화 및 메시지 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/chats` | AI와 채팅하기 | [바로가기](https://www.notion.so/1990abcedc588081a044edfc4156326b?pvs=21) |
| 2 | **`GET`** | `/messages/receivable/:user-id` | 내가 보내는 메시지를 수신할 수 있는 사용자 불러오기 | [바로가기](https://www.notion.so/1990abcedc58806384b5cd91595c7c22?pvs=21) |
| 3 | **`POST`** | `/messages/send` | 새로운 메시지 보내기 | [바로가기](https://www.notion.so/1990abcedc5880ed832ce8fd8428e021?pvs=21) |
| 4 | **`GET`** | `/messages/new?start&end&order` | 새롭게 받은 메시지 가져오기 | [바로가기](https://www.notion.so/1990abcedc5880e8b269e448a92d26a8?pvs=21) |
| 5 | **`GET`** | `/messages/all?start&end&order` | 모든 받은 메시지 가져오기 | [바로가기](https://www.notion.so/1990abcedc58809ab4e9c8e1be2977ab?pvs=21) |
| 6 | **`GET`** | `/messages/sent?start&end&order` | 모든 보낸 메시지 가져오기 | [바로가기](https://www.notion.so/1990abcedc588054a251d6331c97feb4?pvs=21) |
| 7 | **`PATCH`** | `/messages/read/:message-id` | 메시지 읽음 처리하기 | [바로가기](https://www.notion.so/1990abcedc588051ba22fa8109a522a6?pvs=21) |
| 8 | **`PATCH`** | `/messages/read-many` | 다량의 메시지 읽음 처리하기 | [바로가기](https://www.notion.so/19e0abcedc5880e6a0e4e0df0fc9b215?pvs=21) |
| 9 | **`DEL`** | `/messages/delete/:message-id` | 메시지 삭제하기 | [바로가기](https://www.notion.so/1990abcedc58809d9cdfde6c70a2ba7d?pvs=21) |

> **`/notify` - 알림 생성 및 조회 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`POST`** | `/notify` | 새로운 알림 생성하기 | [바로가기](https://www.notion.so/1990abcedc5880118ca1ffb2e4a5a8a4?pvs=21) |
| 2 | **`GET`** | `/notify/new/:family-id?start&end&order` | 새롭게 받은 알림 가져오기 | [바로가기](https://www.notion.so/1990abcedc5880e4beddcbcdf5ef0c24?pvs=21) |
| 3 | **`GET`** | `/notify/all/:family-id?start&end&order` | 모든 알림 가져오기 | [바로가기](https://www.notion.so/1990abcedc5880ebb005ce847d69a088?pvs=21) |
| 4 | **`PATCH`** | `/notify/read/:index` | 알림 읽음 처리하기 | [바로가기](https://www.notion.so/1990abcedc58801d9784f89b14db8cf3?pvs=21) |
| 5 | **`PATCH`** | `/notify/read-many` | 다량의 알림 읽음 처리하기 | [바로가기](https://www.notion.so/19e0abcedc5880839e36df5023d47e5d?pvs=21) |
| 6 | **`DEL`** | `/notify/:index` | 알림 삭제하기 | [바로가기](https://www.notion.so/1990abcedc588044bde4f896f4797c1f?pvs=21) |

> **`/tools` - 부가 기능 부분**
> 

| Order | Method | Path | Description | Details |
| --- | --- | --- | --- | --- |
| 1 | **`GET`** | `/tools/master-region` | 대한민국 광역 자치단체 불러오기 | [바로가기](https://www.notion.so/1930abcedc58807a84f4eaa2bdced7cb?pvs=21) |
| 2 | **`GET`** | `/tools/sub-region` | 대한민국 모든 기초 자치단체 불러오기 | [바로가기](https://www.notion.so/1930abcedc588078acd8cb1c9eb11578?pvs=21) |
| 3 | **`GET`** | `/tools/sub-region/:master-region` | 대한민국 일부 기초 자치단체 불러오기 | [바로가기](https://www.notion.so/1930abcedc58804b80d0ddfda2996543?pvs=21) |
| 4 | **`GET`** | `/tools/ai-server` | AI Process Server 상태 확인하기 | [바로가기](https://www.notion.so/1990abcedc5880d790cec0af61f1ab71?pvs=21) |
| 5 | **`GET`** | `/tools/news` | 대한민국 뉴스 정보 가져오기 | [바로가기](https://www.notion.so/1990abcedc5880309a5bcc75aabe7510?pvs=21) |
| 6 | **`GET`** | `/tools/weather/:user-id` | 대한민국 날씨 정보 가져오기 | [바로가기](https://www.notion.so/1990abcedc5880d99453cc1145cc4e44?pvs=21) |
| 7 | **`GET`** | `/tools/settings/:family-id` | Carebot 설정 정보 가져오기 | [바로가기](https://www.notion.so/1990abcedc5880e98ad5c6619a6afbd5?pvs=21) |
| 8 | **`PATCH`** | `/tools/settings/:family-id` | Carebot 설정 정보 업데이트 하기 | [바로가기](https://www.notion.so/1990abcedc5880bd8b27c5df3e88d410?pvs=21) |
| 9 | **`POST`** | `/tools/background` | 가족 공유 배경화면 추가하기 | [바로가기](https://www.notion.so/19d0abcedc5880498932c26271211d67?pvs=21) |
| 10 | **`GET`** | `/tools/background/:family-id` | 가족 공유 배경화면 가져오기 | [바로가기](https://www.notion.so/19d0abcedc5880b2b72bdfcd72e27c61?pvs=21) |
| 11 | **`DEL`**  | `/tools/background/:family-id/:image-id` | 가족 공유 배경화면 삭제하기 | [바로가기](https://www.notion.so/19d0abcedc5880e2a977fb09666ae412?pvs=21) |