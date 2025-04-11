import { createContext, useContext, useState, useEffect } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { useUserProgressStore } from "./userProgressStore";

const NotificationContext = createContext({
    notifications: [],
    fetchNotifications: () => {},
    addNotification: () => {},
    removeNotification: () => {},
    markNotificationAsRead: () => {},
});

export function useNotificationStore() {
    return useContext(NotificationContext);
}

export default function NotificationProvider({ children }) {
    const { request } = useMainHttp();
    const userProgressStore = useUserProgressStore();
    const [notifications, setNotifications] = useState([]);
    const [userMap, setUserMap] = useState({}); // ID랑 이름 매핑

    async function fetchNotifications() {
        const userId = userProgressStore.loginUserInfo.userInfo?.id;
        if (!userId) {
            console.error("❌ 로그인된 유저 ID가 필요합니다.");
            return;
        }

        try {
            const startTime = "2020-01-01T00:00:00"; // 오래된 날짜부터 모든 메시지 조회
            const endTime = new Date(Date.now() + 9 * 60 * 60 * 1000).toISOString(); // 현재 시간 기준

            const response = await request(`${userProgressStore.DEV_API_URL}/messages/all?start=${startTime}&end=${endTime}&order=desc`);
            if (response.success) {
                // ✅ 받은 메시지 중 내가 수신한 메시지만 필터링
                const receivedMessages = response.data.result.filter(
                    (msg) => msg.to_id === userId
                );

                const receivableResponse = await request(`${userProgressStore.DEV_API_URL}/messages/receivable/${encodeURIComponent(userId)}`);
                if (!receivableResponse.success) {
                    console.error("❌ 수신 가능한 사용자 목록을 가져오는 데 실패:", receivableResponse.error);
                    return;
                }
                const userList = receivableResponse.data.result;
                const userMap = userList.reduce((acc, user) => {
                    acc[user.user_id] = user.name;
                    return acc;
                }, {});
                setUserMap(userMap);

                // ✅ 메시지를 알림 형식으로 변환
                const messageNotifications = receivedMessages.map((msg) => ({
                    id: msg.index,
                    text: `${userMap[msg.from_id] || msg.from_id}님으로부터 새로운 메시지가 도착했습니다: "${msg.content}"`,
                    created_at: msg.created_at,
                    notification_grade: "message",
                    is_read: msg.is_read,
                }));

                // ✅ 알림 상태 업데이트
                setNotifications(messageNotifications);
            } else {
                console.error("❌ 메시지 알림을 가져오는 데 실패했습니다:", response.error);
            }
        } catch (error) {
            console.error("❌ 네트워크 오류 (메시지 알림 조회):", error);
        }
    }

    function addNotification(notification) {
        setNotifications((prev) => [...prev, notification]);
    }

    function removeNotification(id) {
        setNotifications((prev) => prev.filter(notice => notice.id !== id));
    }

    async function markNotificationAsRead(notificationIndex) {
        try {
            const response = await request(
                `${userProgressStore.DEV_API_URL}/notify/read/${notificationIndex}`,
                "PATCH",
                { is_read: true },
                { headers: { "Content-Type": "application/json" } }
            );

            if (response.success && response.data?.result) {
                setDisasterData((prevData) =>
                    prevData.map((item) =>
                        item.index === notificationIndex ? { ...item, is_read: true } : item
                    )
                );
            } else {
                console.error(`❌ 알림(${notificationIndex}) 읽음 처리 실패`, response.error);
            }
        } catch (error) {
            console.error(`❌ 알림(${notificationIndex}) 읽음 처리 중 오류 발생`, error);
        }
    }

    const ctxValue = {
        notifications,
        fetchNotifications,
        addNotification,
        removeNotification,
        markNotificationAsRead
    };

    return (
        <NotificationContext.Provider value={ ctxValue }>
            {children}
        </NotificationContext.Provider>
    );
}