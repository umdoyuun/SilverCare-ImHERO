import { useEffect, useContext, createContext } from "react";

import { UserProgressContext } from "./userProgressStore";
import { HomeStatusContext } from "./homeStatusStore";
import { HealthContext } from "./healthStore";
import { EmergencyContext } from "./emergencyStore";
import { CalendarStoreContext } from "./calendarStore";
import { MessageContext } from "./messageStore";

export const EffectContext = createContext({});

export default function EffectContextProvider({ children }) {
  const ctxValue = {};

  const userProgressStore = useContext(UserProgressContext);
  const homeStatusStore = useContext(HomeStatusContext);
  const healthStore = useContext(HealthContext);
  const emergencyStore = useContext(EmergencyContext);
  const calendarStore = useContext(CalendarStoreContext);
  const messageStore = useContext(MessageContext);

  // 페이지 로드 시 로그인 상태 확인 후 활성화된 사이드 바 상태 가져오기
  useEffect(() => {
    const storedLocalSessionId = localStorage.getItem("session_id");
    const storedSessionSessionId = sessionStorage.getItem("session_id");
    const storedActiveSideBarElem = sessionStorage.getItem(
      "isActiveSideBarElem"
    );

    let sessionId = null;

    if (storedLocalSessionId) {
      try {
        userProgressStore.handleGetSession(storedLocalSessionId);
        userProgressStore.setAutoLogin(true);
      } catch (error) {
        console.error("Error parsing session_id from localStorage:", error);
        localStorage.removeItem("session_id"); // 손상된 데이터 제거
      }
    } else if (storedSessionSessionId) {
      try {
        userProgressStore.handleGetSession(storedSessionSessionId);
      } catch (error) {
        console.error("Error parsing session_id from sessionStorage:", error);
        sessionStorage.removeItem("session_id"); // 손상된 데이터 제거
      }
    }

    // 유저 정보가 있으면 설정
    // if (userInfo) {
    //   console.log(userInfo, 1213123123123213123123);
    //   userProgressStore.setLoginUserInfo({
    //     login: true,
    //     userInfo: userInfo.userInfo,
    //   });

    //   // 사용자 정보 최신화
    //   userProgressStore.handleGetUserInfo(userInfo.userInfo.id);
    // }

    // 사이드바 상태 복원
    if (storedActiveSideBarElem) {
      try {
        userProgressStore.setIsActiveSideBarElem(storedActiveSideBarElem);
      } catch (error) {
        console.error(
          "Error parsing isActiveSideBarElem from sessionStorage:",
          error
        );
        sessionStorage.removeItem("isActiveSideBarElem"); // 데이터 손상 시 제거
      }
    }
  }, []);

  // loginUserInfo가 업데이트된 후에 handleCheckFamilyList 호출
  // loginUserInfo가 상태로 관리되고 있다면, setLoginUserInfo 함수가 비동기적으로 실행되기 때문에 바로 loginUserInfo.userInfo.id에 접근할 때 값이 갱신되지 않았을 수 있습니다.
  // 이는 React의 상태 관리 특성 때문에 발생하는 문제로, 상태가 비동기적으로 업데이트되기 때문에 바로 loginUserInfo 값을 사용할 수 없습니다.
  useEffect(() => {
    const fetchData = async () => {
      if (!userProgressStore.loginUserInfo.login) {
        return;
      } else if (
        userProgressStore.loginUserInfo.login &&
        userProgressStore.loginUserInfo.userInfo.id
      ) {
        // console.log(
        //   `${userProgressStore.loginUserInfo.userInfo.role}유저 가족 정보 요청`
        // );
        if (userProgressStore.loginUserInfo.userInfo.role === "main") {
          await userProgressStore.handleCheckFamilyExist(
            userProgressStore.loginUserInfo.userInfo.id
          );
        } else if (userProgressStore.loginUserInfo.userInfo.role === "sub") {
          await userProgressStore.handleCheckFamilyList();
        }
      }
    };

    fetchData();
  }, [userProgressStore.loginUserInfo.userInfo]);

  useEffect(() => {
    if (!userProgressStore.loginUserInfo.login) {
      return;
    }

    const fetchData = async () => {
      //   console.log("🔄 useEffect 내부 실행됨!", {
      //     member: userProgressStore.memberInfo,
      //     family: userProgressStore.familyInfo,
      //     login: userProgressStore.loginUserInfo,
      //   });

      if (
        userProgressStore.memberInfo.selectedFamilyId ||
        (userProgressStore.familyInfo.familyInfo &&
          userProgressStore.familyInfo.familyInfo.id)
      ) {
        // console.log("API 요청 시작!", {
        //   member: userProgressStore.memberInfo,
        //   family: userProgressStore.familyInfo,
        //   login: userProgressStore.loginUserInfo,
        // });
        await homeStatusStore.handleGetHomeStatus();
        await homeStatusStore.handleGetDeviceStatus();
        await healthStore.handleGetHealthData();
        await healthStore.handleGetActivityStatus();
        await healthStore.handleGetMentalStatus();
        await healthStore.handleGetMentalReports();
        await healthStore.handleGetWeekData();
        await healthStore.handleGetKeywords();
        await emergencyStore.getAllNotifications();
        if (userProgressStore.loginUserInfo.userInfo.role === "sub") {
          await messageStore.handleGetAllMessages();
        }
        // console.log("API 요청 끝!");
      }
    };

    const refreshData = async () => {
      if (
        userProgressStore.memberInfo.selectedFamilyId ||
        userProgressStore.familyInfo.familyInfo.id
      ) {
        // console.log("Refresh 요청 시작!", {
        //   member: userProgressStore.memberInfo,
        //   family: userProgressStore.familyInfo,
        //   login: userProgressStore.loginUserInfo,
        // });
        await homeStatusStore.handleGetHomeStatus();
        await homeStatusStore.handleGetDeviceStatus();
        await healthStore.handleGetHealthData();
        await emergencyStore.handleGetNewNotifications();
        if (userProgressStore.loginUserInfo.userInfo.role === "sub") {
          await messageStore.handleGetAllMessages();
        }
        // console.log("Refresh 요청 끝!");
      }
    };

    // 최초 실행
    fetchData();

    // setInterval에 fetchData 함수를 넘겨야 함
    const intervalId = setInterval(refreshData, 5 * 1000);

    // Cleanup 함수 추가: 컴포넌트가 언마운트 될 때 interval을 클리어
    return () => clearInterval(intervalId);
  }, [
    userProgressStore.memberInfo.selectedFamilyId,
    userProgressStore.familyInfo.familyInfo?.id,
  ]);

  useEffect(() => {
    if (!userProgressStore.loginUserInfo.login) {
      return;
    }

    // console.log("집 상태 정보 요일 별 객체화");
    const fetchData = async () => {
      if (homeStatusStore.homeStatus.length > 0) {
        await calendarStore.groupHomeStatusDataByKSTWithAvgScore(
          homeStatusStore.homeStatus
        );
      } else {
        await calendarStore.groupHomeStatusDataByKSTWithAvgScore({});
      }
    };

    fetchData();
  }, [homeStatusStore.homeStatus]);

  useEffect(() => {
    if (!userProgressStore.loginUserInfo.login) {
      return;
    }

    // console.log("활동 정보 요일 별 객체화");
    const fetchData = async () => {
      if (healthStore.activityStatus.length > 0) {
        await calendarStore.groupDataByKSTWithAvgScore(
          "health",
          healthStore.activityStatus
        );
      } else {
        await calendarStore.groupDataByKSTWithAvgScore("health", {});
      }
    };

    fetchData();
  }, [healthStore.activityStatus]);

  useEffect(() => {
    if (!userProgressStore.loginUserInfo.login) {
      return;
    }

    // console.log("정신 상태 정보 요일 별 객체화");
    const fetchData = async () => {
      if (healthStore.mentalStatus.length > 0) {
        await calendarStore.groupDataByKSTWithAvgScore(
          "mental",
          healthStore.mentalStatus
        );
      } else {
        await calendarStore.groupDataByKSTWithAvgScore("mental", {});
      }
    };

    fetchData();
  }, [healthStore.mentalStatus]);

  useEffect(() => {
    if (!userProgressStore.loginUserInfo.login) {
      return;
    }

    if (emergencyStore.allNotifications.length === 0) {
      return;
    }

    const categorizeData = async () => {
      await emergencyStore.categorizeNotifications(
        emergencyStore.allNotifications
      );
    };

    categorizeData();
  }, [emergencyStore.allNotifications]);

  return (
    <EffectContext.Provider value={ctxValue}>{children}</EffectContext.Provider>
  );
}
