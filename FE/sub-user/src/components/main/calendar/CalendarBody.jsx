import "./Calendar.css";

import { useContext } from "react";
import { CalendarStoreContext } from "../../../store/calendarStore";

export default function CalendarBody() {
  const { daysInMonth, selectedDate, currentDate, schedules } =
    useContext(CalendarStoreContext);

  // console.log(schedules);
  const weeks = ["일", "월", "화", "수", "목", "금", "토"];

  const healthData = schedules.schedules.health;
  const mentalData = schedules.schedules.mental;

  return (
    <div className="calendar-container">
      {/* 요일 표시 */}
      <div className="calendar-day-wrapper">
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
      <div className="calendar-day-wrapper">
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
            <div>
              <span>{date.day}</span>
            </div>
            {/* 캘린더에 일정을 간략하게 표시 */}
            <li className="calendar-day-schedules">
              {healthData[date.date] ? (
                <ul className="health-aver">
                  {healthData[date.date].averageScore}
                </ul>
              ) : (
                <ul className="no-aver">
                  <br />
                </ul>
              )}
              {mentalData[date.date] ? (
                <ul className="mental-aver">
                  {mentalData[date.date].averageScore}
                </ul>
              ) : (
                <ul className="no-aver">
                  <br />
                </ul>
              )}
            </li>
          </div>
        ))}
      </div>
    </div>
  );
}
