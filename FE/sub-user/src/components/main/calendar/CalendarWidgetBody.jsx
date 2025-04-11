import "./Calendar.css";

import { useContext } from "react";
import { CalendarStoreContext } from "../../../store/calendarStore";

export default function CalendarBody() {
  const { daysInMonth, selectedDate, currentDate, schedules } =
    useContext(CalendarStoreContext);

  const weeks = ["일", "월", "화", "수", "목", "금", "토"];

  const healthData = schedules.schedules.health;
  const mentalData = schedules.schedules.mental;

  return (
    <div className="calendar-widget-container">
      {/* 요일 표시 */}
      <div className="calendar-widget-day-wrapper">
        {weeks.map((week, index) => (
          <div
            key={week}
            className={`calendar-item ${index === 0 ? "sunday" : "weekday"}`}
          >
            {week}
          </div>
        ))}
      </div>

      {/* Days */}
      <div className="calendar-widget-day-wrapper">
        {daysInMonth.map((date) => (
          <div
            key={date.date}
            onClick={() => selectedDate.selectDate(date.date)}
            className={`calendar-day 
              ${selectedDate.date === date.date ? "selected-date" : ""}
              ${currentDate.month !== date.month ? "not-current-month" : ""}
              ${date.dayIndexOfWeek === 0 ? "sunday" : ""}
            `}
          >
            <div className="calendar-day-schedules-date">
              <span>{date.day}</span>
            </div>
            {/* 캘린더에 일정을 간략하게 표시 */}
            <div className="calendar-day-schedules-widget">
              {healthData[date.date] ? (
                <div className="health"></div>
              ) : (
                <div className="no" />
              )}
              {mentalData[date.date] ? (
                <div className="mental"></div>
              ) : (
                <div className="no" />
              )}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
