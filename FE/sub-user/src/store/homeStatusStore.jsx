import { useState, createContext, useEffect, useContext } from "react";
import { useHttp } from "../hooks/useHttp";

import { UserProgressContext } from "./userProgressStore";

export const HomeStatusContext = createContext({
  homeStatus: [
    {
      family_id: null,
      reported_at: null,
      temperature: null,
      humidity: null,
      dust_level: null,
      ethanol: null,
      others: { finedust: null, ultrafinedust: null },
    },
  ],
  deviceStatus: {
    family_id: "",
    is_alarm_enabled: false,
    is_camera_enabled: false,
    is_microphone_enabled: false,
    is_driving_enabled: false,
  },
  setHomeStatus: () => {},
  handleGetHomeStatus: () => {},
  handleGetDeviceStatus: () => {},
});

export default function HomeStatusContextProvider({ children }) {
  const { request, loading } = useHttp();

  const userProgressStore = useContext(UserProgressContext);

  const [homeStatus, setHomeStatus] = useState([]);
  const [deviceStatus, setDeviceStatus] = useState({
    family_id: "",
    is_alarm_enabled: false,
    is_camera_enabled: false,
    is_microphone_enabled: false,
    is_driving_enabled: false,
  });

  // useEffect(() => {
  //   try {
  //     if (!userProgressStore.loginUserInfo.login) return;

  //     if (userProgressStore.loginUserInfo.userInfo.role === "sub") {
  //       if (userProgressStore.memberInfo.selectedFamilyId) {
  //         const fetchData = async () => {
  //           await handleGetLatestHomeStatus(
  //             userProgressStore.memberInfo.selectedFamilyId
  //           );
  //         };

  //         fetchData();
  //       }
  //     }
  //   } catch (error) {
  //     console.error("최신 집 정보 가져오기 실패", error);
  //   }
  // }, [
  //   userProgressStore.loginUserInfo.login,
  //   userProgressStore.memberInfo.selectedFamilyId,
  // ]);

  let familyId = "";
  if (userProgressStore.loginUserInfo.userInfo?.role === "sub") {
    familyId = userProgressStore.memberInfo.selectedFamilyId;
  } else if (userProgressStore.loginUserInfo.userInfo?.role === "main") {
    familyId = userProgressStore.familyInfo.familyInfo?.id;
  }

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

  // [
  //   {
  //     "index": 4,
  //     "family_id": "FlcuDLxVC9SolW70",
  //     "reported_at": "2025-02-07T07:31:46",
  //     "temperature": 22.4,
  //     "humidity": 17,
  //     "dust_level": 123.535,
  //     "ethanol": 0.37685,
  //     "others": "{'testData':123, 'ultrafinedust': 1234}"
  //   },
  //   {
  //     "index": 3,
  //     "family_id": "FlcuDLxVC9SolW70",
  //     "reported_at": "2025-02-07T06:15:12",
  //     "temperature": 21.9,
  //     "humidity": 17,
  //     "dust_level": 159.648,
  //     "ethanol": 0.401594,
  //     "others": "{'testData':123, 'ultrafinedust': 1234}"
  //   },
  //   {
  //     "index": 1,
  //     "family_id": "FlcuDLxVC9SolW70",
  //     "reported_at": "2025-02-06T05:36:07",
  //     "temperature": 25.4,
  //     "humidity": 45.9,
  //     "dust_level": 22,
  //     "ethanol": 0.01,
  //     "others": "{'finedust':53, 'ultrafinedust': 23}"
  //   }
  // ]

  async function handleGetHomeStatus(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
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

    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await fetch(
        `${userProgressStore.DEV_API_URL}/status/home/${familyId}?start=${inputStart}&end=${inputEnd}&order=${order}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include",
        }
      );

      const resData = await response.json().catch(() => null);

      if (response.ok) {
        if (resData.message === "Home status retrieved successfully") {
          // others 값이 "{'testData':123, 'ultrafinedust': 1234}"처럼 작은따옴표(' ')로 감싸져 있습니다.
          // JavaScript의 JSON.parse()는 작은따옴표가 아닌 큰따옴표(" ")를 사용해야 정상적으로 파싱됩니다.
          setHomeStatus(
            resData.result.map((data) => {
              if (!data.others) {
                return {
                  ...data,
                  others: {
                    testData: null,
                    ultrafinedust: null,
                  },
                };
              } else {
                return {
                  ...data,
                  others: JSON.parse(data.others.replace(/'/g, '"')), // 작은따옴표를 큰따옴표로 변환
                };
              }
            })
          );
          return {
            success: true,
            data: resData.result,
          };
        } else if (resData.message == "No home status found") {
          console.error("최신 집 내부 정보가 없습니다.", resData.message);
          setHomeStatus([]);
          return {
            success: true,
            data: [],
          };
        }
      } else {
        console.error("최신 집 내부 정보 조회 실패:", response.error);
        setHomeStatus([]);
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
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

  async function handleGetDeviceStatus() {
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
        `${userProgressStore.DEV_API_URL}/tools/settings/${familyId}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include",
        }
      );

      const resData = await response.json().catch(() => null);

      if (response.ok) {
        if (resData.message === "Settings retrieved successfully") {
          setDeviceStatus(resData.result);
          return {
            success: true,
            data: resData.result,
          };
        } else if (resData.message == "No settings found") {
          console.error("디바이스 설정 정보가 없습니다.", resData.message);
          setDeviceStatus({
            family_id: "",
            is_alarm_enabled: false,
            is_camera_enabled: false,
            is_microphone_enabled: false,
            is_driving_enabled: false,
          });
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error("디바이스 설정 정보 조회 실패:", response.error);
        setDeviceStatus({
          family_id: "",
          is_alarm_enabled: false,
          is_camera_enabled: false,
          is_microphone_enabled: false,
          is_driving_enabled: false,
        });
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
      console.error("디바이스 상태 조회 실패:", error);
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
    loading,
    homeStatus,
    deviceStatus,
    setHomeStatus,
    handleGetHomeStatus,
    handleGetDeviceStatus,
  };

  return (
    <HomeStatusContext.Provider value={ctxValue}>
      {children}
    </HomeStatusContext.Provider>
  );
}
