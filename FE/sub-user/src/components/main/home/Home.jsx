import "../Main.css"

import { useContext } from "react"

import { UserProgressContext } from "../../../store/userProgressStore.jsx"

import Widget from "../../widget/Widget.jsx"

import Advertisement from "../advertisement/Advertisement.jsx"

import CalendarWidget from "../calendar/CalendarWidget.jsx"
import EmergencyWidget from "../emergency/EmergencyWidget.jsx"
import KeywordsWidget from "../mental/KeywordsWidget.jsx"
import ToggleGroup from "../../toggle/ToggleGroup.jsx"

// import { UserProgressContext } from "../../../store/userProgressStore.jsx";

export default function Main() {
  const userProgressStore = useContext(UserProgressContext)

  if (!userProgressStore.loginUserInfo.login) {
    return <Advertisement />
  }

  return (
    <div id="home-main">
      {/* <h2 id="main-container-title">HOME</h2> */}

      <h2>홈</h2>
      <div id="page-container">
        <div id="home-widget-container">
          <Widget title="전체 알림" type="emergency">
            <EmergencyWidget />
          </Widget>
        </div>
        <Widget title="캘린더" type="calendar">
          <CalendarWidget />
        </Widget>
        <div id="home-widget-container">
          <Widget title="영웅이 상태">
            <ToggleGroup />
          </Widget>
          <KeywordsWidget />
        </div>
      </div>
    </div>
  )
}
