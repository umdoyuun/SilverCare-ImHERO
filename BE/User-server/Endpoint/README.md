## Carebot Project - API Endpoints

### 개요

Carebot Project는 독거노인을 위한 스마트 생활 도우미 서비스입니다. 이 서비스는 단순한 대화 상대를 넘어, 일상 속에서 동반자가 되어주고, 긴급한 상황에서는 신속한 도움을 제공하는 역할을 합니다.

또한, Carebot은 가족 및 요양보호사가 독거노인의 생활 상태를 원격으로 모니터링할 수 있도록 지원합니다. 대화 내역을 바탕으로 감정 및 심리 상태를 분석하고, 환경 및 활동 데이터를 수집하여 위험 요소를 감지함으로써 보다 안전한 생활을 돕습니다.

이렇게 다양한 기능을 지원하기 위해서는 API의 처리 구조가 복잡해지게 됩니다. 그리고 API에 요청하는 데이터의 형태도 다양할 수 밖에 없습니다. 사용자나 로봇이 보내는 데이터가 많은 만큼 전달하는 데이터의 형태를 정의하는 것이 중요해지게 됩니다.

그리하여 API를 통해 요청을 보낼 때 Body 담아서 보내는 데이터를 Class 형태로 미리 정의하여 잘못된 정보가 입력되지 않도록 방지할 수 있습니다. 잘못된 정보가 입력되는 것을 방지하여 **API의 신뢰성을 높이고, 무분별한 Injection 공격에 대비**할 수 있게 됩니다.

### 기술

<img src="/uploads/cca32fa6cf2f85bb54b812c60262d92f/Python-Dark.svg" width="100" height="100" alt="Python"/>

| **분야** | **사용한 기술** |
| --- | --- |
| Program Language | **Python** 3.11.9 |
| Validation | **Pydantic** 2.10.5 |

### 구성

1. **`Date`** : 날짜를 입력하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`year`** | 연 **[필수]** | `Int` |
    | **`month`** | 월 **[필수]** | `Int` |
    | **`day`** | 일 **[필수]** | `Int` |
2. **`EmailCheck`** : 중복 이메일 검증을 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`email`** | 검증할 이메일 **[필수]** | `String` |
3. **`PasswordCheck`** : 삭제와 같은 중요한 기능을 위해 비밀번호를 검증하는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`password`** | 사용자 검증을 위한 비밀번호 (최소 1글자 이상) **[필수]** | `String` |
4. **`Account`** : 사용자 계정을 생성하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`email`** | 계정 생성에 사용할 이메일 **[필수]** | `String` |
    | **`password`** | 사용자 검증을 위한 비밀번호 (최소 1글자 이상) **[필수]** | `String` |
    | **`role`** | 계정의 종류 (역할) `“main” or “sub” or “system” or “test”` **[필수]** | `String` |
    | **`user_name`** | 사용자의 본명 | `String` |
    | **`birth_date`** | 사용자의 생년월일 | `Dict[String, int]` |
    | **`gender`** | 사용자의 성별 `“male” or “female” or “none”` | `String` |
    | **`address`** | 사용자의 거주지 주소 (기초 자치단체 단위까지 “시군구”) | `String` |
5. **`IDCheck`** : 계정 및 가족 확인을 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`id`** | 계정 및 가족 확인을 위한  ID **[필수]** | `String` |
6. **`Family`** : 가족 생성을 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`main_user`** | Family의 대표가 될 Main User의 ID **[필수]** | `String` |
    | **`family_name`** | Family를 간단하게 나타낼 별명 | `String` |
7. **`FindFamily`** : 개인 정보로 가족을 찾기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`user_name`** | 사용자의 본명 **[필수]** | `String` |
    | **`birth_date`** | 사용자의 생년월일 **[필수]** | `Dict[String, int]` |
    | **`gender`** | 사용자의 성별 `“male” or “female” or “none”` **[필수]** | `String` |
    | **`address`** | 사용자의 거주지 주소 (기초 자치단체 단위까지 “시군구”) **[필수]** | `String` |
8. **`Member`** : 가족 관계를 생성하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`family_id`** | Family를 식별하기 위한 ID **[필수]** | `String` |
    | **`user_id`** | Family에 등록될 Member의 `Account ID` **[필수]** | `String` |
    | **`nickname`** | Main User에게 보여질 Member의 별명 | `String` |
9. **`Login`** : 사용자 계정으로 로그인하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`email`** | 로그인을 위한 사용자 이메일 주소 **[필수]** | `String` |
    | **`password`** | 사용자 검증을 위한 비밀번호 (최소 1글자 이상) **[필수]** | `String` |
10. **`ChangePassword`** : 사용자 계정의 비밀번호를 변경하기 위해 받는 데이터\

    | key | description | type |
    | --- | --- | --- |
    | **`user_id`** | 비밀번호를 변경할 사용자의 ID **[필수]** | `String` |
    | **`current_password`** | 사용자 검증을 위한 비밀번호 (최소 1글자 이상) **[필수]** | `String` |
    | **`new_password`** | 변경할 비밀번호 (최소 1글자 이상) **[필수]** | `String` |
11. **`HomeStatus`** : Carebot 장치에서 집 안의 환경 상태를 보고하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`family-id`** | 정보를 보고할 대상 가족의 ID **[필수]** | `String` |
    | **`temperature`** | 집 안의 온도 (°C) | `Float` |
    | **`humidity`** | 집 안의 습도 (%) | `Float` |
    | **`dust_level`** | 집 안의 미세먼지 농도 (㎍/㎥) | `Float` |
    | **`ethanol`** | 집 안의 에탄올 수치 (ppm) | `Float` |
    | **`others`** | 그 외 추가적인 데이터 | `JSON` |
12. **`HealthStatus`** : Carebot 장치에서 건강 상태를 보고하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`family-id`** | 정보를 보고할 대상 가족의 ID **[필수]** | `String` |
    | **`heart_rate`** | 현재 측정된 분당 심장 박동수 | `Float` |
13. **`ActiveStatus`** : Carebot 장치에서 활동 상태를 보고하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`family-id`** | 정보를 보고할 대상 가족의 ID **[필수]** | `String` |
    | **`score`** | 긍정적인 지표, 부정적인 지표를 종합한 점수
    (`0 ≤ N ≤ 100`) | `Int` |
    | **`action`** | 현재 추적한 활동의 이름 | `String` |
    | **`is_critical`** | 부정적인 지표가 감지되었는지 여부
    (`true of false`) | `Bool` |
    | **`description`** | 활동 내용에 대한 보고서 | `JSON` |
    | **`image_url`** | 활동 내용에 대한 이미지 | `URL` |
14. **`AIChat`** : 독거 노인이 Carebot을 통해 AI와 대화하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`user_id`** | AI Chat을 수행하기 위한 사용자의 ID **[필수]** | `String` |
    | **`message`** | AI에게 보낼 메시지 내용 **[필수]** | `String` |
    | **`session_id`** | 대화 Context 유지를 위한 Session ID
    (첫 대화인 경우 생략 가능) | `UUID` |
15. **`Notification`** : 사용자들에게 보낼 알림을 생성하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`family_id`** | 알림을 보낼 대상의 가족 ID **[필수]** | `String` |
    | **`notification_grade`** | 알림의 종류 (일반 → `info`, 재난 → `warn`, 경보 → `crit`, 구분 없음 → `none`)
    (기본 값 : `none`) | `Enum(’INFO’, ‘WARN’, ‘CRIT’, ‘NONE’)` |
    | **`description`** | 알림 내용에 대한 상세한 설명 | `String` |
    | **`image_url`** | 알림에 포함된 이미지 링크 | `URL` |
16. **`Message`** : 사용자 간의 메시지를 주고 받기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`from_id`** | 메시지를 송신하는 사용자의 ID **[필수]** | `String` |
    | **`to_id`** | 메시지를 수신하는 사용자의 ID **[필수]** | `String` |
    | **`content`** | 메시지에 담을 내용 | `String` |
    | **`image_url`** | 메시지와 함께 보낼 이미지 링크 | `URL` |
17. **`IndexList`** : 여러 개의 메시지와 알림 읽음을 처리하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`index_list`** | Index를 모아둔 리스트 **[필수]** | `List` |
18. **`Settings`** : Carebot의 설정 값을 업데이트하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`is_alarm_enabled`** | 알림 기능이 켜져있는지 여부 | `bool` |
    | **`is_camera_enabled`** | 카메라 기능이 켜져있는지 여부 | `bool` |
    | **`is_microphone_enabled`** | 마이크 기능이 켜져있는지 여부 | `bool` |
    | **`is_driving_enabled`** | 자율 주행 기능이 켜져있는지 여부 | `bool` |
19. **`Background`** : 가족 공유 배경화면 앨범에 새로운 사진을 추가하기 위해 받는 데이터

    | key | description | type |
    | --- | --- | --- |
    | **`family_id`** | 배경화면을 업로드 할 대상의 가족 ID **[필수]** | `String` |
    | **`image_url`** | 배경화면이 업로드 된 링크 | `URL` |