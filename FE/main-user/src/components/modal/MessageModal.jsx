import React from "react";
import { useState, useContext, useEffect } from "react";
import { StoreContext } from "../../store/store.jsx";
import { useMessageStore } from "../../store/messageStore.jsx";
import MessageList from "../message/MessageList.jsx";
import Chatting from "../message/Chatting";
import "./Modal.css";

export default function MessageModal() {
  const store = useContext(StoreContext);
  const [isChatting,setIsChatting] = useState(false);
  const { fetchReceivableUsers } = useMessageStore();

  useEffect(() => {
    console.log("📩 메시지 버튼 클릭됨, fetchReceivableUsers 실행!");
    fetchReceivableUsers();
  }, []);

  const handleBack = () => {
    console.log("🔙 뒤로 가기 버튼 클릭됨, MessageList로 전환");
    setIsChatting(false); // ✅ MessageList로 돌아가기
  };

  if (!useMessageStore) {
    console.error("useMessageStore 오류: MessageProvider가 적용되지 않았습니다!");
  }

  return (
    <div id="modal-body">
      <div id="modal-bar">
        <h2>메세지</h2>
      </div>
      <div id="message-area" className={isChatting ? "chatting-mode" : "list-mode"}>
        {isChatting ? (
            <Chatting onBack={handleBack}/>
        ) : (
            <MessageList onSelectUser={() => setIsChatting(true)} />
        )}
      </div>
      <div className="modal-buttons">
        {isChatting ? ( null
          ) : ( // ✅ 리스트 창에서는 "닫기" 버튼만 표시
          <button onClick={store.handleModalClose} className="button">
            닫기
          </button>
          )}
      </div>
    </div>
  );
}