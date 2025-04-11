import "./Calendar.css"

import { useContext } from "react"
import { CalendarStoreContext } from "../../../store/calendarStore"

import arrowBackIcon from "../../../assets/feature/chevron-left.svg"
import arrowForwardIcon from "../../../assets/feature/chevron-right.svg"

export default function CalendarHeader() {
  const { dispatch, currentDate } = useContext(CalendarStoreContext)

  return (
    <div className="calendar-header-container">
      {/* 연도 전환 리모컨 */}
      <div className="calendar-header-change-button">
        <button onClick={dispatch.handlePrevYear}>
          <img src={arrowBackIcon} alt="prev-year" />
        </button>
        <span>{currentDate.year}</span>
        <button onClick={dispatch.handleNextYear}>
          <img src={arrowForwardIcon} alt="next-year" />
        </button>
      </div>

      {/* 월 전환 리모컨 */}
      <div className="calendar-header-change-button">
        <button onClick={dispatch.handlePrevMonth}>
          <img src={arrowBackIcon} alt="prev-month" />
        </button>
        <span>{currentDate.month}</span>
        <button onClick={dispatch.handleNextMonth}>
          <img src={arrowForwardIcon} alt="next-month" />
        </button>
      </div>
    </div>
  )
}
