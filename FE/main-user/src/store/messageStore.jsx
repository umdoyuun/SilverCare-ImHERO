import React from "react";
import { createContext, useState, useContext, useEffect, useRef } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { useNotificationStore } from "./notificationStore";
import { UserProgressContext } from "./userProgressStore";

const MessageContext = createContext({
    receivableUsers: [],
    selectedUser: null,
    conversations: {},
    isLoading: false,
    fetchReceivableUsers: () => {},
    fetchMessages: () => {},
    selectUser: () => {},
    clearSelectedUser: () => {},
    addMessage: () => {},
});

export function useMessageStore() {
    return useContext(MessageContext);
}

export default function MessageProvider({ children }) {
    const { request } = useMainHttp();
    const userProgressStore = useContext(UserProgressContext);

    const [receivableUsers, setReceivableUsers] = useState([]); // 메시지를 보낼 수 있는 사람 목록
    const [selectedUser, setSelectedUser] = useState(null); // 선택한 대화 상대
    const [conversations, setConversations] = useState({}); // 유저별 대화 저장
    const [isLoading, setIsLoading] = useState(false);

    const notificationStore = useNotificationStore(); // 알림 스토어 가져오기
    const lastCheckedTimeRef = useRef(new Date(Date.now() + 9 * 60 * 60 * 1000).toISOString()); // 마지막 체크한 시간 저장

    let loginUserId = userProgressStore.loginUserInfo.userInfo?.id || "";
    
    async function fetchReceivableUsers() {
        setIsLoading(true);

        try {
            const response = await request(`${userProgressStore.DEV_API_URL}/messages/receivable/${encodeURIComponent(loginUserId)}`);
            const resData = response.data;

            if (response.success) {
                setReceivableUsers(resData.result);
            } else {
                console.error("유저 목록 가져오기 실패:", response.error);
            }
        } catch (error) {
            console.error("네트워크 오류:", error);
        } finally {
            setIsLoading(false);
        }
    }

    async function sendMessageToServer(message) {
        try {
            const response = await request(`${userProgressStore.DEV_API_URL}/messages/send`, "POST", message);
            
            await fetchMessages(message.to_id);

            return response;
        } catch (error) {
            console.error("❌ 메시지 전송 중 오류 발생:", error);
            return { success: false, error };
        }
    }

    async function fetchMessages(selectedUserId) {
        if (!selectedUserId || !loginUserId) {
            console.error("유저 ID가 없습니다.")
            return;
        }
        
        setIsLoading(true);

        try {
            const startTime = "2020-01-01T00:00:00"; // ✅ 모든 메시지를 가져오기 위해 오래된 날짜 설정
            const endTime = new Date(Date.now() + 9 * 60 * 60 * 1000).toISOString();

            const existingMessages = conversations[selectedUserId] || [];

            const receivedResponse = await request(`${userProgressStore.DEV_API_URL}/messages/all?start=${startTime}&end=${endTime}&order=desc`);
            const receivedData = receivedResponse.data;

            const sentResponse = await request(`${userProgressStore.DEV_API_URL}/messages/sent?start=${startTime}&end=${endTime}&order=desc`);
            const sentData = sentResponse.data;

            if (!receivedResponse.success || !sentResponse.success) {
                console.error("❌ 메시지를 가져오지 못했습니다.");
                return;
            }

            /////
            const now = new Date().toISOString(); // 현재 시간 기준
            const newMessages = receivedData.result.filter(
                (msg) => msg.to_id === loginUserId && msg.from_id === selectedUserId && msg.created_at > lastCheckedTimeRef.current
            );

            if (newMessages.length > 0) {
                newMessages.forEach(msg => {
                    notificationStore.addNotification({
                        id: msg.index,
                        text: `${msg.from_id}님으로부터 새로운 메시지가 도착했습니다!`,
                        notification_grade: "message",
                        is_read: false
                    });
                });
            }

            lastCheckedTimeRef.current = now;
            /////

            const receivedMessages = receivedData.result.filter(
                (msg) => msg.to_id === loginUserId && msg.from_id === selectedUserId
            ).map(msg => ({
                ...msg,
                sender: "other"
            }));
    
            const sentMessages = sentData.result.filter(
                (msg) => msg.from_id === loginUserId && msg.to_id === selectedUserId
            ).map(msg => ({
                ...msg,
                sender: "me"
            }));

            if (sentMessages.length === 0) {
                console.warn("⚠️ 서버에서 보낸 메시지를 찾을 수 없습니다.");
            }

            const sortedMessages = [...existingMessages, ...receivedMessages, ...sentMessages]
                .filter((v, i, a) => a.findIndex(t => (t.index === v.index)) === i) // 중복 제거
                .sort((a, b) => new Date(a.created_at) - new Date(b.created_at));

            console.log(`✅ ${selectedUserId}와의 대화 메시지 정리됨`, sortedMessages);

            setConversations((prev) => ({
                ...prev,
                [selectedUserId]: sortedMessages.length > 0 ? sortedMessages : (prev[selectedUserId] || []),
            }));
        } catch (error) {
            console.error("❌ 네트워크 오류: ", error);
        } finally {
            setIsLoading(false);
        }
    }

    // 특정 유저 선택 시 해당 유저의 대화 불러오기
    function selectUser(user) {
        if (!user || !user.user_id) {
            return;
        }

        setSelectedUser(user);

        fetchMessages(user.user_id);
    }

    // 새로운 메시지 추가 (전송/수신)
    function addMessage(selectedUserId, newMessage) {
        setConversations((prev) => ({
            ...prev,
            [selectedUserId]: [...(prev[selectedUserId] || []), newMessage],
        }));
    }

    function clearSelectedUser() {
        setSelectedUser(null);
      }

    useEffect(() => {
        if (!userProgressStore?.loginUserInfo?.userInfo?.id) {
            console.log("⏳ Waiting for login...");
            return;
        }
        
        console.log("🔄 useEffect: fetchReceivableUsers 실행됨");
        fetchReceivableUsers();
    }, [userProgressStore.loginUserInfo.userInfo?.id]);

    const ctxValue = {
        receivableUsers,
        selectedUser,
        conversations,
        isLoading,
        selectUser,
        fetchReceivableUsers,
        sendMessageToServer,
        fetchMessages,
        clearSelectedUser,
        addMessage,
    }

    return (
        <MessageContext.Provider value={ctxValue}>
            {children}
        </MessageContext.Provider>
    );
}