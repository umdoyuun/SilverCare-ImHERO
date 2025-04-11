import "./Nav.css"

import ReactDOM from "react-dom"
import { useState, useEffect, useContext } from "react"
import { useNavigate } from "react-router-dom"

import { UserProgressContext } from "../../store/userProgressStore"
import TopNavSideNavElems from "./TopNavSideElems"

import menuIcon from "../../assets/feature/menu.svg"
import personIcon from "../../assets/feature/user.svg"
import closeIcon from "../../assets/icons/close.svg"
import homeIcon from "../../assets/feature/home.svg"
import calendarIcon from "../../assets/feature/calendar.svg"
import smsIcon from "../../assets/feature/message-circle.svg"
import notificationIcon from "../../assets/feature/bell.svg"
import vitalSignIcon from "../../assets/icons/vital_sign.svg"
import sirenIcon from "../../assets/icons/siren.svg"
import activityIcon from "../../assets/feature/activity.svg"
import settingIcon from "../../assets/icons/settings.svg"

export default function TopNav() {
  const userProgressStore = useContext(UserProgressContext)
  const navigate = useNavigate()

  const [isMobile, setIsMobile] = useState(window.innerWidth <= 720)

  useEffect(() => {
    const handleResize = () => {
      setIsMobile(window.innerWidth <= 720)
    }

    // 이벤트 리스너 등록
    window.addEventListener("resize", handleResize)

    // 컴포넌트가 언마운트될 때 이벤트 리스너 제거
    return () => {
      window.removeEventListener("resize", handleResize)
    }
  }, [])

  // 화면 너비가 720px 이하일 때만 렌더링
  if (!isMobile) return null

  const loginUserInfo = userProgressStore.loginUserInfo

  function handleChangeFamilyId(familyId) {
    userProgressStore.handleChangeFamilyId(familyId)
  }

  return ReactDOM.createPortal(
    <div id="mobile-bar">
      <nav className="top-bar">
        <div className="top-bar-toggle-menu">
          <button onClick={userProgressStore.handleSidebarToggle}>
            <img src={menuIcon} alt="menu" />
          </button>
        </div>
        <h3>{!userProgressStore.isActiveSideBarElem ? "SUNGJOONKING" : userProgressStore.isActiveSideBarElem.toUpperCase()}</h3>
        <div className="top-bar-toggle-menu">
          <button
            onClick={() => {
              navigate("/accounts")
            }}
          >
            <img src={personIcon} alt="userinfo" />
          </button>
        </div>
      </nav>

      <aside className={`top-side-bar ${userProgressStore.sidebarIsOpened ? "open" : "closed"}`}>
        <div className="top-side-bar-header">
          <div className="top-side-bar-header-elem">
            {loginUserInfo.userInfo.role === "sub" && userProgressStore.memberInfo.registerData && (
              <select onChange={(event) => handleChangeFamilyId(event.target.value)}>
                {userProgressStore.memberInfo.registerData.map((info) => (
                  <option key={info.family_id} value={info.family_id}>
                    {info.family_name}
                  </option>
                ))}
              </select>
            )}
            {loginUserInfo.userInfo.role === "sub" && !userProgressStore.memberInfo.registerData && <h5>모임 없음</h5>}
            {loginUserInfo.userInfo.role === "main" && userProgressStore.familyInfo.isExist && <h5>{userProgressStore.familyInfo.familyInfo.family_name}</h5>}
            {loginUserInfo.userInfo.role === "main" && !userProgressStore.familyInfo.isExist && <h5>모임 없음</h5>}
          </div>
          <button onClick={userProgressStore.handleSidebarToggle}>
            <img src={closeIcon} alt="close" />
          </button>
        </div>

        <ul className="top-side-bar-nav">
          <TopNavSideNavElems imgSrc={homeIcon} altSrc="home" identifier="홈" activeIdentifier={userProgressStore.isActiveSideBarElem} onClickElem={userProgressStore.handleActiveSideBarElem} />
          <TopNavSideNavElems
            imgSrc={notificationIcon}
            altSrc="notification"
            identifier="알림"
            activeIdentifier={userProgressStore.isActiveSideBarElem}
            onClickElem={userProgressStore.handleActiveSideBarElem}
          />
          {userProgressStore.loginUserInfo.userInfo.role === "sub" && (
            <TopNavSideNavElems
              imgSrc={smsIcon}
              altSrc="message"
              identifier="메시지"
              activeIdentifier={userProgressStore.isActiveSideBarElem}
              onClickElem={userProgressStore.handleActiveSideBarElem}
            />
          )}
          {/* <TopNavSideNavElems
            imgSrc={sirenIcon}
            altSrc="emergency"
            identifier="EMERGENCY"
            activeIdentifier={userProgressStore.isActiveSideBarElem}
            onClickElem={userProgressStore.handleActiveSideBarElem}
          /> */}
          <TopNavSideNavElems
            imgSrc={calendarIcon}
            altSrc="calendar"
            identifier="캘린더"
            activeIdentifier={userProgressStore.isActiveSideBarElem}
            onClickElem={userProgressStore.handleActiveSideBarElem}
          />
          {userProgressStore.loginUserInfo.userInfo.role === "sub" && (
            <TopNavSideNavElems
              imgSrc={activityIcon}
              altSrc="activity"
              identifier="건강"
              activeIdentifier={userProgressStore.isActiveSideBarElem}
              onClickElem={userProgressStore.handleActiveSideBarElem}
            />
          )}
          {/* <TopNavSideNavElems
            imgSrc={vitalSignIcon}
            altSrc="mental"
            identifier="MENTAL"
            activeIdentifier={userProgressStore.isActiveSideBarElem}
            onClickElem={userProgressStore.handleActiveSideBarElem}
          /> */}
          {/* <TopNavSideNavElems
            imgSrc={settingIcon}
            altSrc="settings"
            identifier="SETTINGS"
            activeIdentifier={userProgressStore.isActiveSideBarElem}
            onClickElem={userProgressStore.handleActiveSideBarElem}
          /> */}
        </ul>
      </aside>
    </div>,
    document.getElementById("mobile-nav")
  )
}
