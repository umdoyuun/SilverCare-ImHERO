import React, { useState, useEffect, useRef, useContext } from "react";
import "./App.css";

import ModalPage from "./components/modal/ModalPage.jsx";
import NavBar from "./components/nav/NavBar.jsx";
import Home from "./components/home/Home.jsx";
import { UserProgressContext } from "./store/userProgressStore.jsx";
import { EnvironmentDataContext } from "./store/environmentData.jsx";
import screenProtector from "./assets/screen-protector.png";
import Login from "./components/login/Login.jsx";

function ScreenSaver({}) {
  return (
    <div className="screensaver-container">
      <img
        src={screenProtector}
        alt="screenprotector"
        className="screen-protector"
      />
    </div>
  );
}

export default function App() {
  const [isScreensaverActive, setIsScreensaverActive] = useState(false);
  const userProgressStore = useContext(UserProgressContext);
  const environmentStore = useContext(EnvironmentDataContext)

  const SCREENSAVER_TIMEOUT = 50000000;
  let timeoutId = null;

  const resetTimer = () => {
    clearTimeout(timeoutId);
    timeoutId = setTimeout(() => {
      setIsScreensaverActive(true);
    }, SCREENSAVER_TIMEOUT);
  };

  useEffect(() => {
    const savedBackground = localStorage.getItem("background");
    if (savedBackground) {
      document.body.style.background = `url(${savedBackground})`;
      document.body.style.backgroundSize = "cover";
      document.body.style.backgroundPosition = "center";
    }

    const handleUserActivity = () => {
      if (isScreensaverActive) {
        setIsScreensaverActive(false);
      }
      resetTimer();
    };

    window.addEventListener("mousemove", handleUserActivity);
    window.addEventListener("keydown", handleUserActivity);
    window.addEventListener("click", handleUserActivity);

    resetTimer();

    // 🔹 드래그 방지 이벤트 추가
    const preventDrag = (event) => event.preventDefault();
    document.addEventListener("dragstart", preventDrag); // 요소 드래그 방지
    document.addEventListener("selectstart", preventDrag); // 텍스트 선택 방지

    return () => {
      clearTimeout(timeoutId);
      window.removeEventListener("mousemove", handleUserActivity);
      window.removeEventListener("keydown", handleUserActivity);
      window.removeEventListener("click", handleUserActivity);
      document.removeEventListener("dragstart", preventDrag);
      document.removeEventListener("selectstart", preventDrag);
    };
  }, [isScreensaverActive]);
  
  useEffect(() => { 
    const initLogin = async () => {
      if (userProgressStore.loginUserInfo.userInfo?.id) {
        await userProgressStore.connectRasp();
        await userProgressStore.getFamiliyId();
      }
    };

    initLogin();
  }, [userProgressStore.loginUserInfo]);

  const prevFamilyInfo = useRef(userProgressStore.familyInfo);

  useEffect(() => {
    const getStatusData = async () => {
      if (
        userProgressStore.familyInfo.isExist && 
        userProgressStore.familyInfo.familyId &&
        (prevFamilyInfo.current.familyId !== userProgressStore.familyInfo.familyId ||
          prevFamilyInfo.current.isExist !== userProgressStore.familyInfo.isExist)
      ) {
        await environmentStore.handleGetLatestEnvironmentData();
        prevFamilyInfo.current = userProgressStore.familyInfo;
      }
    };

    getStatusData();
  }, [userProgressStore.familyInfo])

  return (
    <>
      {isScreensaverActive && (
        <ScreenSaver onDismiss={() => setIsScreensaverActive(false)} />
      )}
      {userProgressStore.loginUserInfo.login ? (
        <main style={{ width: "1024px", height: "600px", overflow: "hidden" }}>
          <ModalPage />
          <NavBar />
          <Home />
        </main>
      ) : (
        <Login />
      )}
    </>
  );
}