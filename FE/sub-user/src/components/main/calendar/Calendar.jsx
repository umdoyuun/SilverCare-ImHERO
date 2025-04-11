import "./Calendar.css";

import { useContext } from "react";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import CalendarHeader from "./CalendarHeader.jsx";
import CalendarBody from "./CalendarBody.jsx";
import CalendarSchedules from "./CalendarSchedules.jsx";

export default function Calendar() {
  const userProgressStore = useContext(UserProgressContext);

  if (!userProgressStore.loginUserInfo.login) {
    return;
  }
  return (
    <div id="calendar-main">
      <h2 id="main-container-title">캘린더</h2>
      <div id="calendar">
        <div id="calendar-left">
          <CalendarHeader />
          <CalendarBody />
        </div>
        <div id="calendar-right">
          <CalendarSchedules />
        </div>
      </div>
      {/* </CalendarStoreContextProvider> */}
    </div>
  );
}
