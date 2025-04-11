import "./Accounts.css"

import { QRCodeCanvas } from "qrcode.react"
import { useContext } from "react"
import { useNavigate } from "react-router-dom"

import { UserProgressContext } from "../../../store/userProgressStore.jsx"

export default function FamilyUserInfo() {
  const userProgressStore = useContext(UserProgressContext)
  const familyUserInfo = userProgressStore.familyInfo

  const navigate = useNavigate()

  function handleShowCreateFamilyUserInfo() {
    userProgressStore.handleOpenModal("create-family-user-info")
  }

  function handleShowUpdateFamilyUserInfo() {
    userProgressStore.handleOpenModal("update-family-user-info")
  }

  function handleShowDeleteFamilyUserInfo() {
    userProgressStore.handleOpenModal("delete-family")
  }

  function handleShowKickMember(memberId) {
    userProgressStore.handleOpenModal("kick-member", memberId)
  }

  return (
    <>
      {!familyUserInfo.isExist && (
        <div id="login-user-info">
          <div className="not-found-family-user-info">
            <h3>등록된 가족 모임 정보가 없습니다.</h3>
            <button onClick={handleShowCreateFamilyUserInfo}>가족 모임 생성</button>
          </div>
        </div>
      )}
      {familyUserInfo.isExist && (
        <div id="login-user-info">
          <div className="family-info-header">
            <div>
              <h3>{familyUserInfo.familyInfo.family_name}네 모임 정보</h3>
              <div className="qr-container">
                <p className="qr-id">{familyUserInfo.familyInfo.id}</p>
                <QRCodeCanvas
                  className="qr-code"
                  onClick={() => navigate(`${window.location.pathname}/register/${familyUserInfo.familyInfo.id}`)}
                  value={`${window.location.origin}/accounts/register/${familyUserInfo.familyInfo.id}`}
                />
              </div>
            </div>
            <div className="family-btn-container">
              <button className="update" onClick={handleShowUpdateFamilyUserInfo}>
                가족 정보 수정
              </button>
              <button className="delete" onClick={handleShowDeleteFamilyUserInfo}>
                가족 정보 삭제
              </button>
            </div>
          </div>

          <div className="family-container">
            {familyUserInfo.familyMember && familyUserInfo.familyMember.length === 0 && <h3>가족 모임에 등록된 멤버가 없습니다.</h3>}
            {familyUserInfo.familyMember && familyUserInfo.familyMember.length > 0 && (
              <>
                <h3>총 {familyUserInfo.familyMember.length}명의 구성원이 등록되어 있습니다.</h3>
                {familyUserInfo.familyMember.map((member) => (
                  <div key={member.id}>
                    <div className="member-container">
                      <table>
                        <tbody>
                          <tr>
                            <td>유저 ID</td>
                            <td>{member.user_id}</td>
                          </tr>
                          <tr>
                            <td>등록 닉네임</td>
                            <td>{member.nickname}</td>
                          </tr>
                        </tbody>
                      </table>

                      <div className="member-btn-container">
                        <button className="delete" onClick={() => handleShowKickMember(member.id)}>
                          추방
                        </button>
                      </div>
                    </div>
                  </div>
                ))}
              </>
            )}
          </div>
        </div>
      )}
    </>
  )
}
