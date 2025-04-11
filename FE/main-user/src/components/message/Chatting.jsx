import React from "react";
import { useEffect, useRef } from "react";
import Message from "./Message.jsx";
import ReplyBar from "./ReplyBar.jsx";
import { useMessageStore } from "../../store/messageStore.jsx";
import { useUserProgressStore } from "../../store/userProgressStore.jsx";
import "./Message.css";

export default function Chatting({ isOpen, onBack }) {
    const { selectedUser, clearSelectedUser, conversations } = useMessageStore();
    const { loginUserInfo } = useUserProgressStore();
    const messageEndRef = useRef(null);
    const messageListRef = useRef(null);
    const isDragging = useRef(false);
    const startY = useRef(0);
    const scrollTop = useRef(0);

    useEffect(() => {
        if (isOpen && selectedUser?.user_id) {
            console.log(`📩 ${selectedUser.user_id}와의 전체 대화 내역 실시간 업데이트 시작`);
            
            // 1초마다 메시지를 새로 불러옴
            const interval = setInterval(() => {
                fetchMessages(selectedUser.user_id);
            }, 1000);

            return () => {
                clearInterval(interval);
                console.log("🚪 채팅 창이 닫혀 실시간 업데이트 중지됨.");
            };
        }
    }, [isOpen, selectedUser]);
    
    useEffect(() => {
        if (messageEndRef.current) {
            messageEndRef.current.scrollIntoView({ behavior: "auto", block: "end" });
        }
    }, [conversations[selectedUser.user_id]]);

    const handleMouseDown = (e) => {
        isDragging.current = true;
        startY.current = e.clientY;
        scrollTop.current = messageListRef.current.scrollTop;
    };
    
    const handleMouseMove = (e) => {
        if (!isDragging.current) return;
        const deltaY = e.clientY - startY.current;
        messageListRef.current.scrollTop = scrollTop.current - deltaY;
    };
    
    const handleMouseUp = () => {
        isDragging.current = false;
    };

    const handleBack = () => {
        clearSelectedUser(); // ✅ 선택된 사용자 해제
        if (typeof onBack === "function") {
            onBack(); // ✅ 부모(`MessageModal`)에서 `setIsChatting(false)` 실행
        } else {
            console.error("❌ onBack is not a function!");
        }
    };
    
    if (!selectedUser || !conversations || !conversations[selectedUser.user_id]) return <p className="loading-message">불러오는 중입니다...</p>;
    
    // ✅ 날짜별 메시지 그룹화 함수
    const formatDate = (dateString) => {
        const date = new Date(dateString); // ✅ 한국 시간 변환 제거

        const today = new Date();
        const yesterday = new Date();
        
        // ✅ 오늘과 어제는 한국 시간 기준으로 판별
        today.setHours(0, 0, 0, 0); 
        yesterday.setDate(today.getDate() - 1);
        yesterday.setHours(0, 0, 0, 0);

        if (date >= today) return "오늘";
        if (date >= yesterday) return "어제";
        
        return date.toISOString().split("T")[0];  // ✅ YYYY-MM-DD 형식
    };

    const groupedMessages = conversations[selectedUser.user_id].reduce((acc, message) => {
        const dateKey = formatDate(message.created_at);
        if (!acc[dateKey]) {
            acc[dateKey] = [];
        }
        acc[dateKey].push(message);
        return acc;
    }, {});

    const handleSendMessage = async (newMessage) => {
        const newMsgObject = {
            index: Date.now() + 9 * 60 * 60 * 1000,
            from_id: loginUserInfo.userInfo.id,
            to_id: selectedUser.user_id,
            created_at: new Date(Date.now() + 9 * 60 * 60 * 1000).toISOString(),
            content: newMessage,
            sender: "me"
        };

        const response = await sendMessageToServer(newMsgObject);

        if (response.success) {
            console.log("✅ 서버에 메시지 저장 완료:", response.data);
        } else {
        console.error("❌ 메시지 전송 실패:", response.error);
        }
    };

    return (
        <div className="message-container">
            <div className="message-header">
                <button className="back-button" onClick={handleBack}>←</button>
                <h2 className="chat-title">{selectedUser.name}</h2>
            </div>
            <div className="message-content" ref={messageListRef}>
                <div 
                    className="message-list"
                    onMouseDown={handleMouseDown}
                    onMouseMove={handleMouseMove}
                    onMouseUp={handleMouseUp}
                    onMouseLeave={handleMouseUp}
                >
                    {(conversations[selectedUser.user_id] && conversations[selectedUser.user_id].length === 0) ? (
                        <p className="no-messages">대화 내역이 없습니다.</p>
                    ) : (
                        Object.entries(groupedMessages).map(([date, messages]) => (
                            <div key={date} className="message-group">
                                <h2 className="message-date">{date}</h2> {/* ✅ 날짜 헤더 추가 */}
                                {messages.map((msg) => (
                                    <Message 
                                        key={msg.index} 
                                        text={msg.content} 
                                        sender={msg.sender} 
                                        time={(() => {
                                            const date = new Date(msg.created_at);
                                            date.setHours(date.getHours() + 9);
                                            const hours = date.getHours();
                                            const minutes = date.getMinutes().toString().padStart(2, "0");
                                            const period = hours >= 12 ? "오후" : "오전";
                                            const formattedHours = hours % 12 || 12;
                                            return `${period} ${formattedHours}:${minutes}`;
                                        })()}
                                        imageUrl={msg.image_url || null}
                                    />
                                ))}
                            </div>
                        ))
                    )}
                    <div ref={messageEndRef} />
                </div>
                <ReplyBar onSend={handleSendMessage} />
            </div>
        </div>
    );
}