import "./Accounts.css"

import { useContext } from "react"

import { UserProgressContext } from "../../../store/userProgressStore.jsx"

export default function MemberUserInfo() {
  const userProgressStore = useContext(UserProgressContext)
  // const loginUserInfo = userProgressStore.loginUserInfo;
  const memberUserInfo = userProgressStore.memberInfo

  function handleShowCreateMemberUserInfo() {
    userProgressStore.handleOpenModal("create-member-user-info")
  }

  function handleShowUpdateMemberUserInfo(id) {
    userProgressStore.handleOpenModal("update-member-user-info", id)
  }

  function handleShowDeleteMemberUserInfo(id) {
    userProgressStore.handleOpenModal("delete-member", id)
  }

  return (
    <>
      {!memberUserInfo.isExist && (
        <div id="login-user-info">
          <div className="not-found-family-user-info">
            {!memberUserInfo.isExist && <h3>연결된 가족 모임 정보가 없습니다.</h3>}
            <button onClick={handleShowCreateMemberUserInfo}>가족 모임 연결</button>
          </div>
        </div>
      )}
      {memberUserInfo.isExist && (
        <div id="login-user-info">
          <div className="login-user-info-header">
            <h3>연결된 가족 모임 정보</h3>
            <button onClick={handleShowCreateMemberUserInfo}>가족 모임 연결</button>
          </div>
          <div className="reg-member-container">
            {memberUserInfo.registerData.map((info) => {
              return (
                <div key={info.id}>
                  <div className="member-container-box">
                    <h3>{info.family_name ? info.family_name : null}</h3>
                    <div className="member-container">
                      <table>
                        <tbody>
                          <tr>
                            <td>모임 ID</td>
                            <td>{info.family_id}</td>
                          </tr>
                          <tr>
                            <td> 등록 닉네임</td>
                            <td>{info.nickname}</td>
                          </tr>
                        </tbody>
                      </table>
                      <div className="member-btn-container">
                        <button className="update" onClick={() => handleShowUpdateMemberUserInfo(info.id)}>
                          닉네임 수정
                        </button>
                        <button className="delete" onClick={() => handleShowDeleteMemberUserInfo(info.id)}>
                          연결 해제
                        </button>
                      </div>
                    </div>
                  </div>
                </div>
              )
            })}
          </div>
        </div>
      )}
    </>
  )
}
