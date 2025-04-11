import { useContext } from "react";
import { Routes, Route, Outlet } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import UserInfo from "./UserInfo.jsx";
import RegisterMemberQr from "./RegisterMemberQr.jsx";

export default function Accounts() {
  const userProgressStore = useContext(UserProgressContext);

  function handleLogout(event) {
    event.preventDefault();

    userProgressStore.handleLogout();
  }

  async function handleShowSignOut() {
    userProgressStore.handleOpenModal("sign-out");
  }

  return (
    <div id="accounts-main">
      <div id="accounts-header">
        <h2>계정</h2>
        <div className="login-form-action">
          <div className="danger-zone">
            <button
              className={
                userProgressStore.autoLogin
                  ? "auto-login-btn-on"
                  : "auto-login-btn"
              }
            >
              <p>
                자동 로그인
                <br />
                {userProgressStore.autoLogin ? "켜짐" : "꺼짐"}
              </p>
            </button>
            <button className="logout-btn" onClick={handleLogout}>
              로그아웃
            </button>
            <button className="logout-btn" onClick={handleShowSignOut}>
              회원 탈퇴
            </button>
          </div>
        </div>
      </div>
      <UserInfo />
    </div>
  );
}
