import { useState, useContext, createContext } from "react";
import { useHttp } from "../hooks/useHttp.js";

import { UserProgressContext } from "./userProgressStore";

export const MessageContext = createContext({
  messagePerson: "",
  messageLog: [],
  setMessagePerson: () => {},
  setMessageLog: () => {},
  handleGetAllReceivedMessages: (inputStart, inputEnd, order) => {},
  handleGetAllSentMessage: (inputStart, inputEnd, order) => {},
  handleGetAllMessages: () => {},
  getNewMessages: (inputStart, inputEnd, order) => {},
  insertPhotoFile: (imageFile) => {},
  handleSendMessage: (content, imageUrl = null) => {},
});

// 임시 키로 사용할 id
let id = 0;

export default function MessageContextProvider({ children }) {
  const userProgressStore = useContext(UserProgressContext);
  const { request, loading } = useHttp();

  const [messagePerson, setMessagePerson] = useState("");
  const [messageLog, setMessageLog] = useState([]);

  // 1년 전부터 현재 시간까지의 범위를 UTC 기준으로 출력하는 함수
  function getOneYearRangeUTC() {
    const now = new Date(); // 현재 시간 (UTC)
    const oneYearAgo = new Date();
    oneYearAgo.setUTCFullYear(now.getUTCFullYear() - 1); // UTC 기준 1년 전

    const formatUTCDate = (date) => date.toISOString().split(".")[0] + "Z"; // 밀리초 제거 후 'Z' 추가

    return {
      start: formatUTCDate(oneYearAgo),
      end: formatUTCDate(now),
    };
  }

  async function handleGetAllReceivedMessages(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/messages/new?start=${inputStart}&end=${inputEnd}&order=${order}`
      );

      const resData = response.data;

      if (response.success) {
        if (
          resData.message === "New received messages retrieved successfully"
        ) {
          // console.log("메시지 수신 성공");
        }
        return { success: true, data: resData.result };
      } else {
        console.error(`메시지 기록 불러오기 실패`, resData.error);
        setMessageLog([]);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      setHealthStatus([]);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetAllSentMessage(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/messages/sent?start=${inputStart}&end=${inputEnd}&order=${order}`
      );

      const resData = response.data;
      // console.log(resData, "sec");
      if (response.success) {
        if (resData.message === "All sent messages retrieved successfully") {
          // console.log("보낸 메시지 정보 수신");
        } else if (resData.message === "No sent messages") {
          // console.log("보낸 메시지 없음");
        }
        return {
          success: true,
          data: resData.result,
        };
      } else {
        console.error(`메시지 기록 불러오기 실패`, resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      setHealthStatus([]);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetAllMessages() {
    try {
      const firstResponse = await handleGetAllReceivedMessages();
      const secondResponse = await handleGetAllSentMessage();
      // console.log(firstResponse, secondResponse, 123);
      if (firstResponse.success && secondResponse.success) {
        // console.log("메시지 기록 수신 성공");

        const receivedMessages = firstResponse.data;
        const sentMessages = secondResponse.data;

        const allMessages = [...receivedMessages, ...sentMessages];

        // new Date(a.created_at) - new Date(b.created_at)
        // → 날짜가 오래된 순서(과거 → 현재)로 정렬됨.
        const sortedMessages = allMessages.sort((a, b) => {
          return new Date(a.created_at) - new Date(b.created_at);
        });

        setMessageLog((prevLog) => {
          if (prevLog.length !== sortedMessages.length) {
            return sortedMessages;
          } else {
            return prevLog;
          }
        });
      }
    } catch (error) {
      console.error("메시지 기록 수신에 실패헀습니다.", error);
    }
  }

  async function getNewMessages(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/messages/new?start=${inputStart}&end=${inputEnd}&order=${order}`
      );

      const resData = response.data;

      if (response.success) {
        if (
          resData.message === "New received messages retrieved successfully"
        ) {
          // console.log("새 메시지 수신!", resData.result);
          await handleGetAllMessages();
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        // console.log(`새 메시지 없음`, resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function insertPhotoFile(formData) {
    try {
      const response = await fetch(
        `${userProgressStore.IMAGE_API_URL}/upload`,
        {
          method: "POST",
          body: formData,
          credentials: "include", // 필요한 경우 포함
        }
      );

      const resData = await response.json().catch(() => null); // JSON 변환 실패 방지

      if (response.ok) {
        if (resData.message === "Image uploaded successfully") {
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error(`이미지 업로드 실패`, resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleSendMessage(content, imageUrl = null) {
    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/messages/send`,
        "POST",
        {
          from_id: userProgressStore.loginUserInfo.userInfo.id,
          to_id: messagePerson,
          content,
          ...(imageUrl && { image_url: imageUrl }), // imageUrl이 있을 때만 추가
        }
      );

      const resData = response.data;

      if (response.success) {
        if (resData.message === "Message sent successfully") {
          // console.log("메시지 전송 성공");
          await handleGetAllMessages();
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error(`메시지 전송 실패`, resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  const ctxValue = {
    messagePerson,
    messageLog,
    setMessagePerson,
    setMessageLog,
    handleGetAllReceivedMessages,
    handleGetAllSentMessage,
    handleGetAllMessages,
    getNewMessages,
    insertPhotoFile,
    handleSendMessage,
  };

  return (
    <MessageContext.Provider value={ctxValue}>
      {children}
    </MessageContext.Provider>
  );
}
