import "./Accounts.css"

import { useContext } from "react"

import { UserProgressContext } from "../../../store/userProgressStore.jsx"

export default function LoginUserInfo() {
  const userProgressStore = useContext(UserProgressContext)
  const loginUserInfo = userProgressStore.loginUserInfo

  function handleShowUpdateUserInfo() {
    userProgressStore.handleOpenModal("update-user-info")
  }

  function handleShowChangePassword() {
    userProgressStore.handleOpenModal("change-password")
  }

  return (
    <div id="login-user-info">
      <div className="login-user-info-header">
        <h3>로그인 유저 정보</h3>
        <div className="login-user-info-buttons">
          <button onClick={handleShowChangePassword}>비밀번호 변경</button>
          <button onClick={handleShowUpdateUserInfo}>회원 정보 수정</button>
        </div>
      </div>
      <div id="user-info-table">
        <table>
          <tbody>
            <tr>
              <td>이름</td>
              <td>{loginUserInfo.userInfo.user_name ? loginUserInfo.userInfo.user_name : "-"}</td>
            </tr>
            <tr>
              <td>성별</td>
              {loginUserInfo.userInfo.gender === "male" && <td>남성</td>}
              {loginUserInfo.userInfo.gender === "female" && <td>여성</td>}
              {!loginUserInfo.userInfo.gender && <td>-</td>}
            </tr>
            <tr>
              <td>생년월일</td>
              <td>{loginUserInfo.userInfo.birth_date ? loginUserInfo.userInfo.birth_date : "-"}</td>
            </tr>
            <tr>
              <td>거주지</td>
              <td>{loginUserInfo.userInfo.address ? loginUserInfo.userInfo.address : "-"}</td>
            </tr>
            <tr>
              <td>가입 이메일</td>
              <td>{loginUserInfo.userInfo.email ? loginUserInfo.userInfo.email : "-"}</td>
            </tr>
            <tr>
              <td>역할</td>
              {loginUserInfo.userInfo.role === "main" && <td>주 사용자</td>}
              {loginUserInfo.userInfo.role === "sub" && <td>보조 사용자</td>}
              {!loginUserInfo.userInfo.role && <td>-</td>}
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  )
}
