import "./Message.css";

import { useRef, useEffect, useContext } from "react";

import { UserProgressContext } from "../../../store/userProgressStore";
import { MessageContext } from "../../../store/messageStore";

export default function MessageChat() {
  const userProgressStore = useContext(UserProgressContext);
  const messageStore = useContext(MessageContext);

  const chatRef = useRef(null);

  // 메시지가 추가될 때마다 스크롤을 맨 아래로 이동
  useEffect(() => {
    if (chatRef.current) {
      chatRef.current.scrollTop = chatRef.current.scrollHeight; // 맨 아래로 스크롤
    }
  }, [messageStore.messageLog, messageStore.messagePerson]);

  const currentPersonMessageLog = messageStore.messageLog.slice();

  return (
    <div id="message-chat">
      {currentPersonMessageLog.length === 0 && (
        <div id="no-chat">
          <span>대화 기록이 없습니다.</span>
        </div>
      )}
      {!messageStore.messagePerson && (
        <div id="no-chat">
          <span>대화 상대를 선택해주세요.</span>
        </div>
      )}
      {messageStore.messagePerson && currentPersonMessageLog.length > 0 && (
        <div id="chat" ref={chatRef}>
          {currentPersonMessageLog.map((log) => {
            // UTC+9 변환
            const createdAtKST = new Date(log.created_at + "Z").toLocaleString(
              "ko-KR",
              {
                timeZone: "Asia/Seoul",
                year: "numeric",
                month: "2-digit",
                day: "2-digit",
                hour: "2-digit",
                minute: "2-digit",
                hour12: true, // 24시간제 사용
              }
            );

            const isEmergency =
              log.content.length > 4 &&
              log.content.substring(0, 4) === "[긴급]";

            return (
              (log.from_id === messageStore.messagePerson &&
                log.to_id === userProgressStore.loginUserInfo.userInfo.id && (
                  <div key={log.index} className="main">
                    <div
                      className={
                        isEmergency ? "emergency-main-chat" : "main-chat"
                      }
                    >
                      {log.image_url && (
                        <img src={log.image_url} alt="이미지" />
                      )}
                      <div>{log.content}</div>
                    </div>
                    <span className="chat-date">{createdAtKST}</span>
                  </div>
                )) ||
              (log.to_id === messageStore.messagePerson &&
                log.from_id === userProgressStore.loginUserInfo.userInfo.id && (
                  <div key={log.index} className="sub">
                    <span className="chat-date">{createdAtKST}</span>
                    <div className="sub-chat">
                      {log.image_url && (
                        <img src={log.image_url} alt="이미지" />
                      )}
                      <div>{log.content}</div>
                    </div>
                  </div>
                )) ||
              null
            );
          })}
        </div>
      )}
    </div>
  );
}
