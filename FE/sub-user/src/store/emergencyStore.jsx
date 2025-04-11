import { useState, useContext, createContext } from "react";
import { useHttp } from "../hooks/useHttp";

import { UserProgressContext } from "./userProgressStore";
import { set } from "date-fns";

export const EmergencyContext = createContext({
  allNotifications: [],
  categorizedNotifications: {
    info: [
      {
        index: 181,
        family_id: "",
        created_at: "",
        notification_grade: "",
        description: "",
        is_read: false,
      },
    ],
    warn: [
      {
        index: 167,
        family_id: "",
        created_at: "",
        notification_grade: "",
        description: {
          MSG_CN: "",
          DST_SE_NM: "",
        },
        is_read: false,
      },
    ],
    crit: [],
  },
  newNotifications: [],
  newCritNotifications: [],
  setAllNotifications: () => {},
  setCategorizedNotifications: () => {},
  getAllNotifications: (order) => {},
  handleGetNewNotifications: (order) => {},
  handleReadNotification: (index) => {},
  categorizeNotifications: (notifications) => {},
  handleClearNewNotifications: () => {},
  handleCheckAllAlert: () => {},
  handleShowAlertLog: () => {},
  handleCheckHomeAlert: () => {},
});

export default function EmergencyContextProvider({ children }) {
  const userProgressStore = useContext(UserProgressContext);

  const { request, loading } = useHttp();

  const [allNotifications, setAllNotifications] = useState([]);
  const [categorizedNotifications, setCategorizedNotifications] = useState({
    info: [],
    warn: [],
    crit: [],
  });
  const [newNotifications, setNewNotifications] = useState([]);
  const [newCritNotifications, setNewCritNotifications] = useState([]);

  let familyId = "";
  if (userProgressStore.loginUserInfo.userInfo?.role === "sub") {
    familyId = userProgressStore.memberInfo.selectedFamilyId;
  } else if (userProgressStore.loginUserInfo.userInfo?.role === "main") {
    familyId = userProgressStore.familyInfo?.familyInfo?.id;
  }

  async function getAllNotifications(order = "desc") {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/notify/all/${familyId}?order=${order}`,
        "GET"
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "All notifications retrieved successfully") {
          const newNotifications = resData.result;

          setAllNotifications(newNotifications);

          // 새로운 일반 알림만 추출
          const unReadNotifications = newNotifications
            .slice()
            .filter((notification) => {
              if (
                notification.is_read === false &&
                notification.notification_grade !== "crit"
              ) {
                return notification;
              }
            });

          setNewNotifications(unReadNotifications);

          // 새로운 긴급 상황 알림만 추출
          const unReadCritNotifications = newNotifications
            .slice()
            .filter((notification) => {
              if (
                notification.is_read === false &&
                notification.notification_grade === "crit"
              ) {
                return notification;
              }
            });

          setNewCritNotifications(unReadCritNotifications);

          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error(response.error);
        return {
          success: false,
          error: {
            type: "get_all_notifications",
            message: "전체 알림을 불러오는데 실패했습니다.",
          },
        };
      }
    } catch (error) {
      console.error(error);
      return {
        success: false,
        error: {
          type: "get_all_notifications",
          message: "전체 알림을 불러오는데 실패했습니다.",
        },
      };
    }
  }

  async function handleGetNewNotifications(order = "desc") {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    try {
      const response = await fetch(
        `${userProgressStore.DEV_API_URL}/notify/new/${familyId}?order=${order}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include",
        }
      );

      const resData = await response.json().catch(() => null); // JSON 변환 실패 방지

      if (response.ok) {
        if (resData.message === "New notification retrieved successfully") {
          const newNotifications = resData.result;

          // 새 알림이 있을 경우에만 상태 업데이트
          if (newNotifications.length > 0) {
            await getAllNotifications();
          }
        }
      } else {
        console.error(response.error);
        return {
          success: false,
          error: {
            type: "get_all_notifications",
            message: "새 알림을 불러오는데 실패했습니다.",
          },
        };
      }
    } catch (error) {
      console.error(error);
      return {
        success: false,
        error: {
          type: "get_all_notifications",
          message: "새 알림을 불러오는데 실패했습니다.",
        },
      };
    }
  }

  // 알림 읽음 처리
  async function handleReadNotification(index) {
    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/notify/read/${index}`,
        "PATCH"
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Notification check read successfully") {
          getAllNotifications();
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error(response.error);
        return {
          success: false,
          error: {
            type: "check_notification",
            message: "알림을 읽음 처리하는데 실패했습니다.",
          },
        };
      }
    } catch (error) {
      console.error(error);
      return {
        success: false,
        error: {
          type: "get_all_notifications",
          message: "전체 알림을 불러오는데 실패했습니다.",
        },
      };
    }
  }

  // 각 알림을 등급에 따라 분류
  function categorizeNotifications(notifications) {
    const categorized = {
      info: [],
      warn: [],
      crit: [],
    };

    notifications.forEach((notification) => {
      const { notification_grade, description } = notification;

      // description이 JSON 문자열이면 객체로 변환
      let parsedDescription;
      try {
        parsedDescription = JSON.parse(description);
      } catch (e) {
        parsedDescription = description; // JSON 파싱 실패 시 원본 유지
      }

      if (categorized.hasOwnProperty(notification_grade)) {
        categorized[notification_grade].push({
          ...notification,
          description: parsedDescription,
        });
      }
    });

    setCategorizedNotifications({ ...categorized });
    return;
  }

  function handleClearNewNotifications() {
    setNewNotifications([]);
  }

  // 일반 알림 전부 읽음 처리.
  async function handleCheckAllAlert() {
    try {
      for (const notification of newNotifications) {
        const response = await request(
          `${userProgressStore.DEV_API_URL}/notify/read/${notification.index}`,
          "PATCH"
        );

        if (response.success) {
          const resData = response.data;

          if (resData.message === "Notification check read successfully") {
            // console.log(`${notification.index} 읽음 처리 완료`);
          }
        }
      }

      // console.log("모든 알림 읽음 처리 완료");
      await getAllNotifications();
    } catch (error) {
      console.error(error);
    }
  }

  // 긴급 상황 알림 기록 모달 호출
  function handleShowAlertLog() {
    userProgressStore.handleOpenModal("emergency-alert-log");
  }

  function handleCheckHomeAlert() {
    return setHomeNotifications((prevAlerts) => {
      return prevAlerts.map((prevAlert) => {
        return {
          ...prevAlert,
          check: true,
        };
      });
    });
  }

  const ctxValue = {
    loading,
    allNotifications,
    categorizedNotifications,
    newNotifications,
    newCritNotifications,
    setAllNotifications,
    setCategorizedNotifications,
    getAllNotifications,
    handleGetNewNotifications,
    handleReadNotification,
    categorizeNotifications,
    handleClearNewNotifications,
    handleCheckAllAlert,
    handleShowAlertLog,
    handleCheckHomeAlert,
  };

  return (
    <EmergencyContext.Provider value={ctxValue}>
      {children}
    </EmergencyContext.Provider>
  );
}
