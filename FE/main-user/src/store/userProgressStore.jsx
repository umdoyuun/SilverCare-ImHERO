import { useState, useEffect, createContext, useContext } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { getEnvironments } from "./environmentsStore.jsx";
import { StoreContext } from "./store.jsx"

export const UserProgressContext = createContext({
  loginUserInfo: {
    login: false,
    userInfo: undefined,
  },
  familyInfo: {
    isExist: false,
    familyId: undefined,
  },
  DEV_API_URL: "",
  MAIN_API_URL: "",
  DEV_KEY: "",
  MAIN_KEY: "",
  connectRasp: () => {},
  handleLogin: () => {},
  getFamiliyId: () => {},
  handleLogout: () => {},
});

export function useUserProgressStore() {
  return useContext(UserProgressContext);
}

export default function UserProgressContextProvider({ children }) {
  const mainStore = useContext(StoreContext)

  const { request, loading } = useMainHttp();
  const [loginUserInfo, setLoginUserInfo] = useState({
    login: false,
    userInfo: undefined,
  });
  const [familyInfo, setFamilyInfo] = useState({
    isExist: false,
    familyId: undefined,
    familyMember: undefined,
  });

  // 세션 저장을 위해 추가된 사항
  useEffect(() => {
    const storedLoginUserInfo = sessionStorage.getItem("loginUserInfo");

    if (storedLoginUserInfo) {
        const parsedUserInfo = JSON.parse(storedLoginUserInfo);

        if (!loginUserInfo.login || loginUserInfo.userInfo?.id !== parsedUserInfo.id) {
          setLoginUserInfo({
              login: true,
              userInfo: parsedUserInfo,
          });

          console.log("🔄 세션에서 로그인 정보 복구:", parsedUserInfo);
        }
    }
  })

    // ======================================================================
  // env 관련
  let DEV_API_URL = import.meta.env.VITE_DEV_API;
  let MAIN_API_URL = import.meta.env.VITE_MAIN_API;
  let IMAGE_API_URL = import.meta.env.VITE_IMAGE_API;
  let DEV_KEY = import.meta.env.VITE_DEV_KEY;
  let MAIN_KEY = import.meta.env.VITE_MAIN_KEY;

  if (DEV_API_URL === undefined) DEV_API_URL = getEnvironments("DEV_API_URL");
  if (MAIN_API_URL === undefined)
    MAIN_API_URL = getEnvironments("MAIN_API_URL");
  if (IMAGE_API_URL === undefined)
    IMAGE_API_URL = getEnvironments("IMAGE_API_URL");
  if (DEV_KEY === undefined) DEV_KEY = getEnvironments("DEV_KEY");
  if (MAIN_KEY === undefined) MAIN_KEY = getEnvironments("MAIN_KEY");
  // ======================================================================

  function handleUpdateSessionLoginInfo(userInfo) {
    setLoginUserInfo({
      login: true,
      userInfo,
    });

    // 로그인 정보 저장
    sessionStorage.setItem("loginUserInfo", JSON.stringify(userInfo));
  }

  // 로그인
  async function handleLogin(email, password) {
    try {
      const response = await request(`${DEV_API_URL}/auth/login`, "POST", {
        email,
        password,
      });

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Login successful") {
          console.log("로그인 성공", resData);

          const existingSession = sessionStorage.getItem("loginUserInfo");
          if (!existingSession || JSON.parse(existingSession).id !== resData.result.user_data.id) {
              sessionStorage.setItem("loginUserInfo", JSON.stringify(resData.result.user_data));
              setLoginUserInfo({
                  login: true,
                  userInfo: resData.result.user_data,
              });
          }

          await connectRasp(email, password);

          return { success: true, data: resData.user_id };
        }
      } else {
        console.error("로그인 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
      console.error("네트워크 오류 또는 기타 예외:", error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function connectRasp(email, password) {
    try {
      const response = await request(`http://70.12.247.214:8001/api/login`, "POST", {
        email,
        password,
      });
  
      if (response.success) {
        console.log("✅ 라즈베리파이 로그인 성공:", response);
      } else {
        console.log("❌ 라즈베리파이 로그인 실패:", response);
      }
    } catch (error) {
      console.error("❌ 라즈베리파이 네트워크 에러", error);
    }
  }

  async function getFamiliyId() {
    if (!loginUserInfo.login) {
      return
    }

    try {
      const response = await request(`${DEV_API_URL}/families/check-exist`, 'POST', {id: loginUserInfo.userInfo.id})
    
      if (response.success) {
        const resData = response.data
        if (resData.message === "Family exists") {
          console.log('가족 모임 존재')
          setFamilyInfo((prev) => {
            return {
              ...prev,
              isExist:true,
              familyId:resData.result.family_id
            }
          })
        }
      } else {
        console.error('가족 모임이 없습니다.')
        setFamilyInfo({
          isExist: false,
          familyId: undefined,
          familyMember: undefined
        })
      }
    
    } catch (error) {
      console.error(error)
    }
  }

  // 로그아웃
  async function handleLogout() {
    try {
      const response = await request(`${DEV_API_URL}/auth/logout`, "POST");

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Logout successful") {
          console.log("로그아웃 성공");

          setLoginUserInfo({
            login: false,
            userInfo: undefined,
          });

          // 세션 스토리지에서 로그인 정보 삭제
          sessionStorage.removeItem("loginUserInfo");
          sessionStorage.removeItem("session_id");

          // 기본 경로로 이동
          window.location.href = "/";

          // 모달 초기화
          await mainStore.handleModalClose();

          // 기기 활성화 요소 초기화 (동기 처리)
          sessionStorage.removeItem("isActiveSideBarElem");

          return { success: true, data: resData };
        }
      } else {
        console.error("로그아웃 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error?.type || "unknown_error",
            message: response.error?.message || "알 수 없는 오류 발생",
            input: response.error?.input,
          },
        };
      }
    } catch (error) {
      console.error("네트워크 오류 또는 기타 예외:", error);
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
    loginUserInfo,
    familyInfo,
    DEV_API_URL,
    MAIN_API_URL,
    DEV_KEY,
    MAIN_KEY,
    connectRasp,
    setLoginUserInfo,
    setFamilyInfo,
    handleLogin,
    getFamiliyId,
    handleLogout,
  };

  return (
    <UserProgressContext.Provider value={ctxValue}>
      {children}
    </UserProgressContext.Provider>
  );
}