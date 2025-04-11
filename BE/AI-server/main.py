from fastapi import FastAPI, HTTPException, Depends, Query, Path
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session, sessionmaker
from sqlalchemy import create_engine, event, exc, select, text

import os
import logging
import asyncio
from datetime import datetime, date
from dotenv import load_dotenv
from typing import Optional
from openai import OpenAI
from pydantic import BaseModel

from services.weather import WeatherService
from services.chat import ChatService
from services.emotion import EmotionService
from services.disaster import DisasterService
from services.news import NewsService
from services.mental_health import MentalHealthService
from services.message_service import MessageService
from utils.cache import CacheManager
from models import Base, Account, ChatSession, ChatHistory, MentalStatus, FallDetection, Family, ChatKeywords, MentalReport, MemberRelations

from utils.timezone_utils import get_kst_today, to_utc_start_of_day, to_utc, get_kst_now

from typing import List

# 환경 변수 로드
load_dotenv()

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('app.log')
    ]
)
logger = logging.getLogger(__name__)

# FastAPI 앱 초기화
app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# DB 연결 설정
DATABASE_URL = f"mysql+pymysql://{os.getenv('DB_USER')}:{os.getenv('DB_PASSWORD')}@{os.getenv('DB_HOST')}:3306/S12P11A102"

class DBSessionManager:
    def __init__(self, db_url):
        self.engine = create_engine(
            db_url,
            pool_recycle=120,  
            pool_pre_ping=True,
            pool_use_lifo=True,
            echo_pool=True 
        )
        self.SessionLocal = sessionmaker(
            bind=self.engine,
            autocommit=False,
            autoflush=False
        )
        self._db = None
        self.initialize_db()

    def initialize_db(self):
        if self._db:
            try:
                self._db.close()
            except:
                pass
        self._db = self.SessionLocal()

    @property
    def db(self):
        if self._db is None:
            self.initialize_db()
        return self._db

db_manager = DBSessionManager(DATABASE_URL)
db = db_manager.db

# 서비스 초기화
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
weather_service = WeatherService(api_key=os.getenv("WEATHER_API_KEY"))
disaster_service = DisasterService(api_key=os.getenv("DISASTER_API_KEY"))
news_service = NewsService(api_key=os.getenv("NEWS_API_KEY"))
cache_manager = CacheManager()

class ChatRequest(BaseModel):
    user_id: str
    session_id: Optional[str] = None
    user_message: str

class ChatResponse(BaseModel):
    session_id: str
    bot_message: str

class PeriodRequest(BaseModel):
    start_date: datetime
    end_date: datetime

class MessageRequest(BaseModel):
    from_id: str
    content: str

class MessageRequestone(BaseModel):
    from_id: str
    to_id: str
    content: str

class MessageResponse(BaseModel):
    index: int
    sender_nickname: str
    content: str
    created_at: datetime
    is_read: bool

async def update_cache_periodically():
    while True:
        try:
            users = db_manager.db.query(Account).all()
            for user in users:
                if user.address:
                    weather_data = await weather_service.get_weather_for_user(user.id, db_manager.db)
                    if weather_data:
                        cache_manager.set_weather(user.id, weather_data)
            await asyncio.sleep(3600)  # 1시간 대기 
        except Exception as e:
            logger.error(f"Cache update error: {str(e)}")

@app.get("/heartbeat", status_code=200)
async def heartbeat():
    return {"status": "ok"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        chat_service = ChatService(openai_client, db)
        response = await chat_service.process_chat(request.user_id, request.user_message, request.session_id)
        db.commit()
        return response
    except Exception as e:
        db.rollback()
        logger.error(f"Chat processing error: {str(e)}")

        try:
           chat_service = ChatService(openai_client, db)
           response = await chat_service.process_chat(request.user_id, request.user_message, request.session_id)
           db.commit()
           return response
        except Exception as retry_e:
           db.rollback()
           raise HTTPException(status_code=500, detail=str(retry_e))
    
@app.get("/weather/{user_id}")
async def get_weather(user_id: str):
    try:
        # 캐시 확인
        cached_data = cache_manager.get_weather(user_id)
        if cached_data:
            return cached_data

        # 새로운 날씨 데이터 조회
        weather_data = await weather_service.get_weather_for_user(user_id, db)
        if not weather_data:
            raise HTTPException(status_code=404, detail="날씨 정보를 찾을 수 없습니다")

        # 캐시 업데이트
        cache_manager.set_weather(user_id, weather_data)
        print(weather_data)
        db.commit()
        return weather_data

    except Exception as e:
        db.rollback()
        logger.error(f"Weather error: {str(e)}")
        try:
        # 캐시 확인
            cached_data = cache_manager.get_weather(user_id)
            if cached_data:
                return cached_data

            # 새로운 날씨 데이터 조회
            weather_data = await weather_service.get_weather_for_user(user_id, db)
            if not weather_data:
                raise HTTPException(status_code=404, detail="날씨 정보를 찾을 수 없습니다")

            # 캐시 업데이트
            cache_manager.set_weather(user_id, weather_data)
            print(weather_data)
            db.commit()
            return weather_data

        except Exception as e:
            db.rollback()
            logger.error(f"Weather error: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

@app.get("/news")
async def get_news():
    try:
        news_data = await news_service.get_news()
        if not news_data:
            raise HTTPException(status_code=404, detail="뉴스 정보를 찾을 수 없습니다")
        return news_data
    except Exception as e:
        logger.error(f"News error: {str(e)}")
        try:
            news_data = await news_service.get_news()
            if not news_data:
                raise HTTPException(status_code=404, detail="뉴스 정보를 찾을 수 없습니다")
            return news_data
        except Exception as e:
            logger.error(f"News error: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

async def update_notifications_periodically():
    while True:
        try:
            await disaster_service.update_disaster_notifications(db_manager.db)
            await asyncio.sleep(300)  
        except Exception as e:
            logger.error(f"Notification update error: {str(e)}")

@app.post("/generate-emotional-report/{family_id}")
async def generate_emotional_report(family_id: str):
    try:
        emotion_service = EmotionService(openai_client, db)
        report = await emotion_service.generate_report(family_id)
        
        if not report:
            raise HTTPException(status_code=404, detail="감정 분석을 위한 대화 내용이 충분하지 않습니다")
        
        db.commit()
        return report

    except Exception as e:
        db.rollback()
        logger.error(f"Emotion report error: {str(e)}")
        try:
            emotion_service = EmotionService(openai_client, db)
            report = await emotion_service.generate_report(family_id)
            
            if not report:
                raise HTTPException(status_code=404, detail="감정 분석을 위한 대화 내용이 충분하지 않습니다")
            
            db.commit()
            return report

        except Exception as e:
            db.rollback()
            logger.error(f"Emotion report error: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))
    
@app.post("/generate-emotional-report/period/{family_id}")
async def gnerate_periodic_report(
    family_id: str,
    period: PeriodRequest
):
    try:
        emotion_service = EmotionService(openai_client, db)
        report = await emotion_service.generate_periodic_report(family_id, period.start_date, period.end_date)
        db.commit()
        return report
    
    except Exception as e:
        db.rollback()
        logger.error(f"Periodic report error: {str(e)}")
        try:
            emotion_service = EmotionService(openai_client, db)
            report = await emotion_service.generate_periodic_report(family_id, period.start_date, period.end_date)
            db.commit()
            return report
        
        except Exception as e:
            db.rollback()
            logger.error(f"Periodic report error: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

@app.get("/generate-keyword/{family_id}")
async def generate_keywords(family_id: str):
    try:
        today = get_kst_today()
        today_utc = to_utc_start_of_day(today)

        chat_history = db.query(ChatHistory)\
            .join(Family, Family.main_user == ChatHistory.user_id)\
            .filter(Family.id == family_id)\
            .filter(ChatHistory.created_at >= today_utc)\
            .all()
            
        if not chat_history:
            return {"keywords": ["대화를 시작해보세요"]}

        messages = []
        for chat in chat_history:
            messages.append({"role": "user", "content": chat.user_message})
            messages.append({"role": "assistant", "content": chat.bot_message})

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {
                    "role": "system",
                    "content": """대화 기록을 기반으로 user가 관심을 가지고 있는 것 같은
                    키워드 5개를 추출해서 쉼표(,)로 구분된 형태로 응답해주세요.
                    예시: 건강,취미,가족,운동,음식"""
                },
                {"role": "user", "content": str(messages)}
            ]
        )

        keywords = response.choices[0].message.content.strip()
        keyword_list = keywords.split(',')
        keyword_list = [keyword.strip() for keyword in keyword_list]

        existing_keywords = db.query(ChatKeywords)\
            .filter(ChatKeywords.family_id == family_id)\
            .filter(ChatKeywords.created_at >= datetime.now().date())\
            .first()
        
        if existing_keywords:
            existing_keywords.keywords = keywords
        
        else:
            new_keywords = ChatKeywords(
                family_id=family_id,
                keywords=keywords
            )
            db.add(new_keywords)

        db.commit()

        
        return {"keywords": keyword_list}
    
    except Exception as e:
        db.rollback()
        logger.error(f"키워드 생성 오류: {str(e)}")
        try:
            today = get_kst_today()
            today_utc = to_utc_start_of_day(today)

            chat_history = db.query(ChatHistory)\
                .join(Family, Family.main_user == ChatHistory.user_id)\
                .filter(Family.id == family_id)\
                .filter(ChatHistory.created_at >= today_utc)\
                .all()
                
            if not chat_history:
                return {"keywords": ["대화를 시작해보세요"]}

            messages = []
            for chat in chat_history:
                messages.append({"role": "user", "content": chat.user_message})
                messages.append({"role": "assistant", "content": chat.bot_message})

            response = openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": """대화 기록을 기반으로 user가 관심을 가지고 있는 것 같은
                        키워드 5개를 추출해서 쉼표(,)로 구분된 형태로 응답해주세요.
                        예시: 건강,취미,가족,운동,음식"""
                    },
                    {"role": "user", "content": str(messages)}
                ]
            )

            keywords = response.choices[0].message.content.strip()
            keyword_list = keywords.split(',')
            keyword_list = [keyword.strip() for keyword in keyword_list]

            existing_keywords = db.query(ChatKeywords)\
                .filter(ChatKeywords.family_id == family_id)\
                .filter(ChatKeywords.created_at >= datetime.now().date())\
                .first()
            
            if existing_keywords:
                existing_keywords.keywords = keywords
            
            else:
                new_keywords = ChatKeywords(
                    family_id=family_id,
                    keywords=keywords
                )
                db.add(new_keywords)

            db.commit()

            
            return {"keywords": keyword_list}
        
        except Exception as e:
            db.rollback()
            logger.error(f"키워드 생성 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

@app.post('/analyze-mental-health/{family_id}')
async def analyze_mental_health(
    family_id: str,
    period: Optional[PeriodRequest] = None
):
    """
    - 오늘 하루 분석시에는 POST /analyze-mental-health/{family_id}로 요청
    - 특정 기간 분석시에는 body에 start_date, end_date 담아서 보내면 됨
    """
    try:
        mental_health_service = MentalHealthService(openai_client, db)
        result = await mental_health_service.analyze_mental_health(
            family_id,
            period.start_date if period else None,
            period.end_date if period else None
        )
        db.commit()
        return result
    except ValueError as ve:
        db.rollback()
        raise HTTPException(status_code=404, detail=str(ve))
    except Exception as e:
        db.rollback()
        logger.error(f'Mental health analysis error: {str(e)}')
        raise HTTPException(status_code=404, detail=str(e))

@app.post("/chat/message")
async def send_message(request: MessageRequest):
    try:
        message_service = MessageService(db)
        success, count = await message_service.broadcast_message(
            from_id=request.from_id,
            content=request.content
        )
        
        if success:
            db.commit()
            return {
                "status": "success",
                "message": f"메시지가 {count}명의 가족 구성원에게 전송되었습니다"
            }
        db.rollback()
        raise HTTPException(status_code=500, detail="메시지 전송에 실패했습니다")
    except Exception as e:
        db.rollback()
        logger.error(f"메시지 전송 오류: {str(e)}")
        try:
            message_service = MessageService(db)
            success, count = await message_service.broadcast_message(
                from_id=request.from_id,
                content=request.content
            )
            
            if success:
                db.commit()
                return {
                    "status": "success",
                    "message": f"메시지가 {count}명의 가족 구성원에게 전송되었습니다"
                }
            db.rollback()
            raise HTTPException(status_code=500, detail="메시지 전송에 실패했습니다")
        except Exception as e:
            db.rollback()
            logger.error(f"메시지 전송 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))
    
@app.post("/chat/message/single")
async def send_single_message(request: MessageRequestone):
    try:
        message_service = MessageService(db)
        success = await message_service.send_message(
            from_id=request.from_id,
            to_id=request.to_id,
            content=request.content
        )
        
        if success:
            db.commit()
            return {"status": "success", "message": "메시지가 전송되었습니다"}
        db.rollback()
        raise HTTPException(status_code=500, detail="메시지 전송에 실패했습니다")
    
    except Exception as e:
        db.rollback()
        logger.error(f"메시지 전송 오류: {str(e)}")
        try:
            message_service = MessageService(db)
            success = await message_service.send_message(
                from_id=request.from_id,
                to_id=request.to_id,
                content=request.content
            )
            
            if success:
                db.commit()
                return {"status": "success", "message": "메시지가 전송되었습니다"}
            db.rollback()
            raise HTTPException(status_code=500, detail="메시지 전송에 실패했습니다")
        
        except Exception as e:
            db.rollback()
            logger.error(f"메시지 전송 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

@app.get("/chat/messages/{user_id}", response_model=List[MessageResponse])
async def get_unread_messages(user_id: str):
    try:
        message_service = MessageService(db)
        messages = message_service.get_unread_messages(user_id)
        
        response_messages = []
        for msg in messages:
            # 발신자의 가족 정보 조회
            main_user_family = db.query(Family)\
                .filter(Family.main_user == msg.to_id)\
                .first()
                
            if main_user_family:
                member_relation = db.query(MemberRelations)\
                    .filter(
                        MemberRelations.family_id == main_user_family.id,
                        MemberRelations.user_id == msg.from_id
                    ).first()
                nickname = member_relation.nickname if member_relation else "가족"
            else:
                nickname = "가족"
                
            response_messages.append(MessageResponse(
                index=msg.index,
                sender_nickname=nickname,
                content=msg.content,
                created_at=msg.created_at,
                is_read=bool(msg.is_read)
            ))
        
        db.commit()
        return response_messages
        
    except Exception as e:
        db.rollback()
        logger.error(f"메시지 조회 오류: {str(e)}")
        try:
            message_service = MessageService(db)
            messages = message_service.get_unread_messages(user_id)
            
            response_messages = []
            for msg in messages:
                # 발신자의 가족 정보 조회
                main_user_family = db.query(Family)\
                    .filter(Family.main_user == msg.to_id)\
                    .first()
                    
                if main_user_family:
                    member_relation = db.query(MemberRelations)\
                        .filter(
                            MemberRelations.family_id == main_user_family.id,
                            MemberRelations.user_id == msg.from_id
                        ).first()
                    nickname = member_relation.nickname if member_relation else "가족"
                else:
                    nickname = "가족"
                    
                response_messages.append(MessageResponse(
                    index=msg.index,
                    sender_nickname=nickname,
                    content=msg.content,
                    created_at=msg.created_at,
                    is_read=bool(msg.is_read)
                ))
            
            db.commit()
            return response_messages
            
        except Exception as e:
            db.rollback()
            logger.error(f"메시지 조회 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

@app.post("/chat/messages/read/{message_id}")
async def mark_message_as_read(message_id: int):
    try:
        message_service = MessageService(db)
        if await message_service.mark_as_read(message_id):
            db.commit()
            return {"status": "success", "message": "메시지를 읽음 처리했습니다"}
        db.rollback()
        raise HTTPException(status_code=404, detail="메시지를 찾을 수 없습니다")
    except Exception as e:
        db.rollback()
        logger.error(f"메시지 읽음 처리 오류: {str(e)}")
        try:
            message_service = MessageService(db)
            if await message_service.mark_as_read(message_id):
                db.commit()
                return {"status": "success", "message": "메시지를 읽음 처리했습니다"}
            db.rollback()
            raise HTTPException(status_code=404, detail="메시지를 찾을 수 없습니다")
        except Exception as e:
            db.rollback()
            logger.error(f"메시지 읽음 처리 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

@app.get('/family/main/{user_id}')
async def get_family_by_main_user(user_id: str):
    """메인 사용자 ID로 가족 정보 조회"""
    try:
        family = db.query(Family).filter(Family.main_user == user_id).first()

        if not family:
            raise HTTPException(status_code=404, detail="가족 정보를 찾을 수 없습니다")
        
        db.commit()
        return {
            "id": family.id,  
            "main_user": family.main_user
            
        }
        
    except Exception as e:
        db.rollback()
        logger.error(f"가족 정보 조회 오류: {str(e)}")
        try:
            family = db.query(Family).filter(Family.main_user == user_id).first()

            if not family:
                raise HTTPException(status_code=404, detail="가족 정보를 찾을 수 없습니다")
            
            db.commit()
            return {
                "id": family.id,  
                "main_user": family.main_user
                
            }
            
        except Exception as e:
            db.rollback()
            logger.error(f"가족 정보 조회 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))
    
  
@app.get('/family/{family_id}/members')
async def get_family_members(family_id: str):
    try:
        members = db.query(MemberRelations)\
            .filter(MemberRelations.family_id == family_id)\
            .all()
        
        if not members:
            raise HTTPException(status_code=404, detail='가족 구성원을 찾을 수 없습니다')
        
        member_list = [
            {
                "user_id": member.user_id,
                "nickname": member.nickname,
            } for member in members
        ]
        
        db.commit()
        return member_list
    
    except Exception as e:
        db.rollback()
        logger.error(f"가족 구성원 조회 오류: {str(e)}")
        try:
            members = db.query(MemberRelations)\
                .filter(MemberRelations.family_id == family_id)\
                .all()
            
            if not members:
                raise HTTPException(status_code=404, detail='가족 구성원을 찾을 수 없습니다')
            
            member_list = [
                {
                    "user_id": member.user_id,
                    "nickname": member.nickname,
                } for member in members
            ]
            
            db.commit()
            return member_list
        
        except Exception as e:
            db.rollback()
            logger.error(f"가족 구성원 조회 오류: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))


async def schedule_news_updates():
    while True:
        await news_service.update_news()
        await asyncio.sleep(3600)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(update_notifications_periodically())
    asyncio.create_task(update_cache_periodically())
    asyncio.create_task(schedule_news_updates()) 
    logger.info("Application startup completed")

@app.on_event("shutdown")
async def shutdown_event():
    db_manager.engine.dispose()  


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)