from datetime import datetime, timedelta, timezone
import logging
from typing import Optional, Dict
from openai import OpenAI
from sqlalchemy.orm import Session
import uuid

from models import ChatSession, ChatHistory

from utils.timezone_utils import get_kst_now, to_utc

logger = logging.getLogger(__name__)

class ChatService:
    def __init__(self, openai_client: OpenAI, db: Session):
        self.client = openai_client
        self.db = db

    async def process_chat(self, user_id: str, user_message: str, session_id: Optional[str] = None) -> Dict:
        try:
            session_id = await self._get_or_create_session(user_id, session_id)
            history = await self._get_chat_history(session_id)
            
            messages = self._prepare_messages(history, user_message)
            response = await self._get_gpt_response(messages)
            
            await self._save_chat_history(session_id, user_id, user_message, response)
            
            return {
                "session_id": session_id,
                "bot_message": response
            }
        except Exception as e:
            logger.error(f"Chat processing error: {str(e)}")
            raise

    async def _get_or_create_session(self, user_id: str, session_id: Optional[str]) -> str:
        try:
            current_time = get_kst_now()
            
            if not session_id:
                new_session = ChatSession(
                    uid=str(uuid.uuid4()),
                    user_id=user_id,
                    created_at=to_utc(current_time),
                    last_active=to_utc(current_time)
                )
                self.db.add(new_session)
                self.db.commit()
                return new_session.uid

            session = self.db.query(ChatSession).filter(ChatSession.uid == session_id).first()
            if session:
                session.last_active = to_utc(current_time)
                self.db.commit()
                return session.uid

            new_session = ChatSession(
                uid=session_id,
                user_id=user_id,
                created_at=to_utc(current_time),
                last_active=to_utc(current_time)
            )
            self.db.add(new_session)
            self.db.commit()
            return new_session.uid

        except Exception as e:
            self.db.rollback()
            logger.error(f"Session creation error: {str(e)}")
            raise

    async def _get_chat_history(self, session_id: str):
        return self.db.query(ChatHistory)\
            .filter(ChatHistory.session_id == session_id)\
            .order_by(ChatHistory.created_at.asc())\
            .limit(5)\
            .all()

    async def _get_gpt_response(self, messages: list) -> str:
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            max_tokens=120,
            temperature=0.7
        )
        return response.choices[0].message.content.strip()

    def _prepare_messages(self, history, user_message: str) -> list:
        messages = [{
            "role": "system",
            "content": """
            이모티몬은 넣지 말고 텍스트로만 대화하세요.
            3문장 이상으로 대답하지 마세요.
            당신은 독거노인을 위한 따뜻한 돌봄 도우미 영웅이 입니다. 
            노인분들의 외로움을 달래주고 편안한 대화를 나누며, 
            존댓말을 사용하고 너무 어렵지 않은 쉬운 말로 대화하세요. 
            노인분들의 감정을 이해하고 공감하는 대화를 하세요.
            """
        }]
        
        for h in history:
            messages.append({"role": "user", "content": h.user_message})
            messages.append({"role": "assistant", "content": h.bot_message})
        
        messages.append({"role": "user", "content": user_message})
        return messages

    async def _save_chat_history(self, session_id: str, user_id: str, user_message: str, bot_message: str):
        try:
            chat_history = ChatHistory(
                session_id=session_id,
                user_id=user_id,
                user_message=user_message,
                bot_message=bot_message,
            )
            self.db.add(chat_history)
            self.db.commit()
        except Exception as e:
            self.db.rollback()
            logger.error(f"Chat history save error: {str(e)}")
            raise