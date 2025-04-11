from typing import Tuple, List
from sqlalchemy.orm import Session
import logging
from models import Message, Family, Account, MemberRelations

logger = logging.getLogger(__name__)

class MessageService:
    def __init__(self, db: Session):
        self.db = db

    async def send_message(self, from_id: str, to_id: str, content: str) -> bool:
        """특정 사용자에게 메시지 전송"""
        try:
            main_user_family = self.db.query(Family).filter(Family.main_user == to_id).first()

            if main_user_family:
                member_relation = self.db.query(MemberRelations)\
                    .filter(
                        MemberRelations.family_id == main_user_family.id,
                        MemberRelations.user_id == from_id
                    ).first()
                if not member_relation:
                    logger.error(f"가족 구성원 관계 정보를 찾을 수 없음: {from_id}")
                    return False
            
            message = Message(
                from_id=from_id,
                to_id=to_id,
                content=content,
                is_read=0
            )
            self.db.add(message)
            self.db.commit()
            return True
        except Exception as e:
            self.db.rollback()
            logger.error(f"메시지 전송 오류: {str(e)}")
            return False

    async def broadcast_message(self, from_id: str, content: str) -> Tuple[bool, int]:
        """메시지 전송 및 저장"""
        try:
            # 발신자의 가족 정보 조회
            family = self.db.query(Family).filter(Family.main_user == from_id).first()
            
            if not family:
                logger.error(f"가족 정보를 찾을 수 없음: {from_id}")
                return False, 0

            # SUB 역할의 가족 구성원들
            sub_members = self.db.query(Account, MemberRelations)\
                .join(MemberRelations, MemberRelations.user_id == Account.id)\
                .filter(
                    Account.role == 'SUB',
                    MemberRelations.family_id == family.id
                ).all()

            message_count = 0
            for member, _ in sub_members:
                message = Message(
                    from_id=from_id,
                    to_id=member.id,
                    content=content,
                    is_read=0
                )
                self.db.add(message)
                message_count += 1

            self.db.commit()
            return True, message_count

        except Exception as e:
            self.db.rollback()
            logger.error(f"메시지 전송 오류: {str(e)}")
            return False, 0
        
    def get_unread_messages(self, user_id: str) -> List[Message]:
        """읽지 않은 메시지 조회"""
        try:
            return self.db.query(Message)\
                .filter(
                    Message.to_id == user_id,
                    Message.is_read == 0
                )\
                .order_by(Message.created_at.asc())\
                .all()
        except Exception as e:
            logger.error(f"메시지 조회 오류: {str(e)}")
            return []

    async def mark_as_read(self, message_id: int) -> bool:
        """메시지 읽음 처리"""
        try:
            message = self.db.query(Message).filter(Message.index == message_id).first()
            if message:
                message.is_read = 1
                self.db.commit()
                return True
            return False
        except Exception as e:
            self.db.rollback()
            logger.error(f"메시지 읽음 처리 오류: {str(e)}")
            return False

    def format_message_for_tts(self, message: Message) -> str:
        """TTS 음성 출력을 위한 메시지 포맷팅"""
        try:
            # 메시지 발신자의 가족 ID 조회
            main_user_family = self.db.query(Family)\
                .filter(Family.main_user == message.to_id)\
                .first()
            
            if main_user_family:
                member_relation = self.db.query(MemberRelations)\
                    .filter(
                        MemberRelations.family_id == main_user_family.id,
                        MemberRelations.user_id == message.from_id
                    ).first()
                
                if member_relation and member_relation.nickname:
                    return f"{member_relation.nickname}님이 보낸 메시지입니다. {message.content}"
            
            return f"가족이 보낸 메시지입니다. {message.content}"
            
        except Exception as e:
            logger.error(f"메시지 포맷팅 오류: {str(e)}")
            return message.content