## Carebot Project - External Connector

### 개요

Carebot Project는 독거노인을 위한 스마트 생활 도우미 서비스입니다. 이 서비스는 단순한 대화 상대를 넘어, 일상 속에서 동반자가 되어주고, 긴급한 상황에서는 신속한 도움을 제공하는 역할을 합니다.

또한, Carebot은 가족 및 요양보호사가 독거노인의 생활 상태를 원격으로 모니터링할 수 있도록 지원합니다. 대화 내역을 바탕으로 감정 및 심리 상태를 분석하고, 환경 및 활동 데이터를 수집하여 위험 요소를 감지함으로써 보다 안전한 생활을 돕습니다.

간단히 날씨나 뉴스 정보를 수집하고, 외부 LLM API를 통해서 감정 및 심리 상태를 분석하는 서비스는 수 많은 요청이 들어오는 API Server와 별개로 운영하도록 설계하였습니다. 이는 외부 서비스가 적용되는 Server를 별도로 분리함으로써, 외부 요인으로 인해 **전체 서비스가 영향 받는 상황을 최소화** 하기 위함입니다.

### 기술

<img src="/uploads/cca32fa6cf2f85bb54b812c60262d92f/Python-Dark.svg" width="100" height="100" alt="Python"/>

| **분야** | **사용한 기술** |
| --- | --- |
| Program Language | **Python** 3.11.9 |
| Connector | **HTTPX** 0.28.1 |

### 구성

1. **check_connection()** → `/heartbeat`
    
    AI Process Server가 정상적으로 동작하고 있는지 확인하는 기능
    
    - Parameters
        - 없음
    - Return
        - **200** OK
        - **408** Request Timeout
        - **500** Internal Server Error
2. **request_mental_status(family_id)** → `/generate-emotional-report/{family-id}`
    
    AI Chat 기록을 기반으로 하루 단위의 감정 상태의 보고서를 생성하는 기능
    
    - Parameters
        - `family_id` : 대상 가족의 ID
    - Return
        - **200** OK + Reports 내용 (_DB에 저장됨_)
        - **404** Not Found
        - **500** Internal Server Error
3. **request_mental_reports(family_id, start, end)** → `/generate-emotional-report/period/{family-id}`
    
    AI Chat 기록을 기반으로 장기간의 감정 상태의 보고서를 생성하는 기능
    
    - Parameters
        - `family_id` : 대상 가족의 ID
        - `start` : 보고서를 생성할 AI Chat 기록의 시작
        - `end` : 보고서를 생성할 AI Chat 기록의 끝
    - Return
        - **200** OK + Reports 내용 (_DB에 저장됨_)
        - **500** Internal Server Error
4. **request_conversation_keywords(family_id)** → `/generate-keyword/{family-id}`
    
    AI Chat 기록을 기반으로 하루의 대표 Keywords를 추출하는 기능
    
    - Parameters
        - family_id : 대상 가족의 ID
    - Return
        - **200** OK + Keywords (_DB에 저장되지 않음_)
        - **500** Internal Server Error
5. **request_psychology_report(family_id, start, end)** → `/analyze-mental-health/{family-id}`
    
    감정 상태와 AI Chat 기록을 기반으로 심리 상태 보고서를 생성하는 기능
    
    - Parameters
        - `family_id` : 대상 가족의 ID
        - `start` : 보고서를 생성할 AI Chat 기록의 시작
        - `end` : 보고서를 생성할 AI Chat 기록의 끝
    - Return
        - 200 OK + Reports 내용 (_DB에 저장되지 않음_)
        - **404** Not Found
        - **500** Internal Server Error
6. **talk_with_ai(user_id, message, session_id)** → `/chat/message`
    
    OpenAI를 이용해서 AI와 대화를 수행하는 기능
    
    - Parameters
        - `user_id` : 사용자 계정의 ID
        - `message` : AI에 보낼 메시지
        - `session_id` : 메시지 Context를 유지하기 위한 식별 정보
    - Return
        - **200** OK + 답변 내용 (_DB에 저장됨_)
        - **500** Internal Server Error
7. **korean_news()** → `/news`
    
    하루 전의 한국 주요 뉴스를 불러오는 기능
    
    - Parameters
        - 없음
    - Return
        - **200** OK + 뉴스 내용 (_DB에 저장됨_)
        - **404** Not Found
        - **500** Internal Server Error
8. **korean_weather(user_id)** → `/weather/{user-id}`
    
    사용자의 지역 정보를 이용해 날씨를 불러오는 기능
    
    - Parameters
        - `user_id` : 사용자 계정의 ID
    - Return
        - **200** OK + 날씨 내용 (_DB에 저장되지 않음_)
        - **404** Not Found
        - **500** Internal Server Error