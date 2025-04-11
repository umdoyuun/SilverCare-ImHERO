import logging
from datetime import datetime, timedelta, timezone
from typing import Dict, List
from sqlalchemy.orm import Session
import json
from datetime import date

from models import MentalStatus, ChatHistory, ChatSession, Family, MentalReport

from utils.timezone_utils import get_kst_today, to_utc_start_of_day, to_utc, get_kst_now, to_kst

logger = logging.getLogger(__name__)


class EmotionService:
    def __init__(self, openai_client, db: Session):
        self.client = openai_client
        self.db = db

    # 사용자 대화 내용을 분석하여 감정 상태를 분석하고 결과를 반환
    async def generate_report(self, family_id: str) -> Dict:
        try:
            today = get_kst_today()
            today_utc = to_utc_start_of_day(today)

            conversations = self.db.query(ChatHistory)\
                .join(Family, Family.main_user == ChatHistory.user_id)\
                .filter(Family.id == family_id)\
                .filter(ChatHistory.created_at >= today_utc)\
                .all()

            if not conversations:
                logger.warning(f"No conversations found for user {family_id}")
                return None

            # 감정 분석
            analysis_result = await self._analyze_emotions(conversations)
            
            # 결과 저장
            mental_status = MentalStatus(
                family_id=family_id,
                score=self._calculate_score(analysis_result),
                is_critical=self._is_critical(analysis_result),
                description=json.dumps(analysis_result, ensure_ascii=False)
            )
            
            try:
                self.db.add(mental_status)
                self.db.commit()
            except Exception as e:
                self.db.rollback()
                logger.error(f"Failed to save mental status: {str(e)}")
                raise
            
            return analysis_result

        except Exception as e:
            logger.error(f"Emotion analysis error: {str(e)}")
            raise

    async def _analyze_emotions(self, conversations: List[ChatHistory]) -> Dict:
        base_format = {
            "overall_emotional_state": "",
            "emotional_insights": "",
            "time_based_emotions": {},
            "recommendations": []
        }

        try:
            messages = [{
                "role": "system",
                "content": """다음 형식의 JSON으로만 응답하세요. 다른 말은 하지 말고 오직 JSON만 반환하세요:
                {
                    "overall_emotional_state": "한 문장으로 된 감정 상태",
                    "emotional_insights": "2-3문장으로 된 통찰",
                    "time_based_emotions": {
                        "HH:MM": ["긍정적/부정적", 점수(0-100), "설명"],
                        "HH:MM": ["긍정적/부정적", 점수(0-100), "설명"]
                    },
                    "recommendations": [
                        "권장사항1",
                        "권장사항2"
                    ]
                }"""
            }]

            conv_text = "\n".join([
                f"시간: {c.created_at.strftime('%H:%M')}\n"
                f"사용자: {c.user_message}\n"
                f"봇: {c.bot_message}\n"
                for c in conversations
            ])
            
            messages.append({"role": "user", "content": conv_text})

            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                temperature=0.1,  
                max_tokens=1000,  
                presence_penalty=-0.5, 
                frequency_penalty=-0.5
            )

            try:
                # 응답에서 JSON 부분만 추출
                content = response.choices[0].message.content.strip()
                # 앞뒤 불필요한 문자 제거
                if content.startswith('```json'):
                    content = content[7:-3]
                elif content.startswith('```'):
                    content = content[3:-3]
                
                result = json.loads(content)
                
                # 필수 필드 검증 및 보정
                for key in base_format.keys():
                    if key not in result:
                        result[key] = base_format[key]
                
                return result

            except json.JSONDecodeError:
                logger.error("JSON 파싱 실패, 기본 형식으로 변환")
                time_str = to_kst(conversations[-1].created_at.strftime('%H:%M'))
                return {
                    "overall_emotional_state": "감정 상태 분석이 필요합니다.",
                    "emotional_insights": "더 많은 대화가 필요합니다.",
                    "time_based_emotions": {
                        time_str: ["중립", 50, "감정 상태를 분석할 수 없습니다."]
                    },
                    "recommendations": ["더 많은 대화를 나누어보세요."]
                }

        except Exception as e:
            logger.error(f"감정 분석 중 오류: {str(e)}")
            raise

    def _calculate_score(self, analysis_result: Dict) -> int:
        try:
            emotions = analysis_result.get('time_based_emotions', {})
            scores = [emotion[1] for emotion in emotions.values()]
            return int(sum(scores) / len(scores)) if scores else 0
        except Exception as e:
            logger.error(f"Score calculation error: {str(e)}")
            return 0

    def _is_critical(self, analysis_result: Dict) -> int:
        score = self._calculate_score(analysis_result)
        return 1 if score <= 30 else 0
    
    async def generate_periodic_report(self, family_id: str, start_date: datetime, end_date: datetime) -> Dict:
        try:
            start_utc = to_utc_start_of_day(start_date.date())
            end_utc = to_utc_start_of_day(end_date.date() + timedelta(days=1))

            reports = self.db.query(MentalStatus)\
                .filter(MentalStatus.family_id == family_id)\
                .filter(MentalStatus.reported_at >= start_utc)\
                .filter(MentalStatus.reported_at <= end_utc)\
                .all()
            
            if not reports:
                return {
                    'average_score': 0,
                    'critical_days': 0,
                    'best_day': None,
                    'worst_day': None,
                    'improvement_needed': False,
                    'summary': "아직 충분한 데이터가 없습니다."
                }

            # 분석 결과 계산
            average_score = round(sum(r.score for r in reports) / len(reports), 1)
            critical_days = len([r for r in reports if r.is_critical == 1])
            best_day = max(reports, key=lambda x: x.score).reported_at
            worst_day = min(reports, key=lambda x: x.score).reported_at
            improvement_needed = len([r for r in reports if r.score < 50]) / len(reports) > 0.3
            summary = self._generate_period_summary(reports)

            mental_report = MentalReport(
                family_id=family_id,
                start_time=datetime.combine(start_date, datetime.min.time()),
                end_time=datetime.combine(end_date, datetime.max.time()),
                average_score=average_score,
                critical_days=critical_days,
                best_day=best_day.date(),
                worst_day=worst_day.date(),
                improvement_needed=improvement_needed,
                summary=summary
            )
            
            self.db.add(mental_report)
            self.db.commit()
            
            # 결과 반환
            return {
                'average_score': average_score,
                'critical_days': critical_days,
                'best_day': best_day.isoformat(),
                'worst_day': worst_day.isoformat(),
                'improvement_needed': improvement_needed,
                'summary': summary
            }

        except Exception as e:
            self.db.rollback()
            logger.error(f"Periodic report generation error: {str(e)}")
            raise

    def _generate_period_summary(self, reports: List[MentalStatus]) -> str:
        try:
            avg_score = round(sum(r.score for r in reports) / len(reports),1)
            critical_days = len([r for r in reports if r.is_critical == 1])
            best_day = max(reports, key=lambda x: x.score).reported_at.strftime('%Y-%m-%d')
            worst_day = min(reports, key=lambda x: x.score).reported_at.strftime('%Y-%m-%d')

            if avg_score >= 70:
                return f"최근 {len(reports)}일 간 평균 감정 점수는 {avg_score}점으로 높은 편입니다. " \
                    f"위기 상황은 {critical_days}일 발생했으며, 최고 점수는 {best_day}, 최저 점수는 {worst_day}입니다."\
                    "전반적으로 매우 안정적인 감정 상태를 유지하고 있습니다."
            elif avg_score >= 50:
                return f"최근 {len(reports)}일 간 평균 감정 점수는 {avg_score}점으로 중간 정도입니다. " \
                    f"위기 상황은 {critical_days}일 발생했으며, 최고 점수는 {best_day}, 최저 점수는 {worst_day}입니다."\
                    "대체로 양호한 감정 상태를 보이고 있습니다."
            else:
                return f"최근 {len(reports)}일 간 평균 감정 점수는 {avg_score}점으로 낮은 편입니다. " \
                    f"위기 상황은 {critical_days}일 발생했으며, 최고 점수는 {best_day}, 최저 점수는 {worst_day}입니다."\
                    "감정 상태 개션이 필요해 보입니다. 더 많은 대화와 교류를 추천드립니다."
        except Exception as e:
            logger.error(f"Period summary generation error: {str(e)}")
            return "요약 정보를 생성할 수 없습니다."