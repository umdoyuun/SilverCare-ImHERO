# services/mental_health.py
from datetime import datetime, timedelta, timezone
from typing import Dict, Optional, List
from sqlalchemy.orm import Session
import logging

from models import ChatHistory, Family
from openai import OpenAI
import json

from utils.timezone_utils import get_kst_today, to_utc_start_of_day, to_utc_end_of_day, get_kst_now

logger = logging.getLogger(__name__)

class MentalHealthService:
    def __init__(self, openai_client: OpenAI, db: Session):
        self.client = openai_client
        self.db = db

    async def analyze_mental_patterns(self, conversations: List[ChatHistory]) -> Dict:
        """대화의 언어학적 패턴을 분석하여 정신 건강 상태를 평가"""
        try:
            messages = [{
                "role": "system",
                "content": """사용자의 대화 내용을 언어학적으로 분석하여 정신 건강 상태를 평가하세요.
                ex) 언어 패턴이 이상하다면 뇌졸증을 의심할 수 있고 인지력과 대화 맥략이 떨어진다면 치매를 의심할 수 있음.
                다음 요소들을 중점적으로 분석하고 JSON 형식으로만 응답하세요:

                1. 언어 패턴 분석:
                   - 단어 선택과 반복
                   - 문장 구조의 일관성
                   - 표현의 명확성

                2. 대화 맥락 분석:
                   - 주제 전환의 자연스러움
                   - 대화의 연속성
                   - 맥락 이해도

                3. 인지적 특성:
                   - 사고의 명확성
                   - 현실 인식
                   - 판단력

                응답 형식:
                {
                    "language_patterns": {
                        "score": 0-100,
                        "word_choice": "분석 내용",
                        "sentence_structure": "분석 내용",
                        "expression_clarity": "분석 내용"
                    },
                    "contextual_analysis": {
                        "score": 0-100,
                        "topic_flow": "분석 내용",
                        "context_understanding": "분석 내용"
                    },
                    "cognitive_state": {
                        "score": 0-100,
                        "clarity": "분석 내용",
                        "reality_perception": "분석 내용"
                    },
                    "overall_assessment": {
                        "total_score": 0-100,
                        "risk_level": "low/medium/high",
                        "concerns": ["우려사항1", "우려사항2"],
                        "strengths": ["강점1", "강점2"]
                    },
                    "recommendations": [
                        "권장사항1",
                        "권장사항2"
                    ]
                }"""
            }]

            # 대화 내용을 시간 순서대로 정렬하여 문자열로 변환
            conv_text = "\n".join([
                f"시간: {c.created_at.strftime('%H:%M')}\n"
                f"사용자: {c.user_message}\n"
                for c in sorted(conversations, key=lambda x: x.created_at)
            ])
            
            messages.append({"role": "user", "content": conv_text})

            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=messages,
                temperature=0.2,
                max_tokens=1000
            )

            result = response.choices[0].message.content
            return json.loads(result)

        except Exception as e:
            logger.error(f"Mental pattern analysis error: {str(e)}")
            raise

    async def analyze_mental_health(
        self,
        family_id: str,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None
    ) -> Dict:
        """종합적인 정신 건강 분석 수행"""
        try:
            # 대화 내용 조회
            query = self.db.query(ChatHistory)\
                .join(Family, Family.main_user == ChatHistory.user_id)\
                .filter(Family.id == family_id)

            if start_date and end_date:
                start_utc = to_utc_start_of_day(start_date.date())
                end_utc = to_utc_end_of_day(end_date.date())

                query = query.filter(
                    ChatHistory.created_at >= start_utc,
                    ChatHistory.created_at <= end_utc
                )
            else:
                today = get_kst_today()
                today_utc = to_utc_start_of_day(today)
                query = query.filter(ChatHistory.created_at >= today_utc)

            conversations = query.all()
            if not conversations:
                raise ValueError("분석할 대화 내용이 없습니다")

            # 정신 건강 패턴 분석
            analysis_result = await self.analyze_mental_patterns(conversations)

            # 추가 정보 포함
            analysis_result.update({
                "analysis_period": {
                    "start": start_date.isoformat() if start_date else get_kst_today().isoformat(),
                    "end": end_date.isoformat() if end_date else get_kst_today().isoformat()
                },
                "data_stats": {
                    "total_conversations": len(conversations),
                    "analysis_type": "periodic" if start_date else "daily"
                }
            })

            return analysis_result

        except Exception as e:
            logger.error(f"Mental health analysis error: {str(e)}")
            raise