import { useContext, useRef } from "react";
import { CalendarStoreContext } from "../../store/calendarStore";

export default function CalendarSchedules() {
  const { selectedDate, schedules } = useContext(CalendarStoreContext);

  const inputScheduleRef = useRef(""); // ref 초기화

  function handleSubmitSchedule(event) {
    event.preventDefault();

    const newSchedule = inputScheduleRef.current.value; // 입력값 가져오기
    if (newSchedule.trim()) {
      schedules.addSchedule(selectedDate.date, newSchedule); // 새 일정 추가
      inputScheduleRef.current.value = ""; // 입력값 초기화
    }
  }

  return (
    <>
      <div className="calender-schedule">
        {/* 해당 날짜 일정 출력 */}
        <ul>
          {schedules.schedules[selectedDate.date] ? (
            schedules.schedules[selectedDate.date].map((schedule, index) => (
              <li key={index}>- {schedule}</li>
            ))
          ) : (
            <li>해당 일은 아무 일정이 없습니다.</li>
          )}
        </ul>
        <div>
          <a className="calender-schedule-date">{selectedDate.date}</a>
        </div>
      </div>

      {/* 일정 추가 input 그룹 */}
      <form onSubmit={handleSubmitSchedule} className="calendar-form">
        <input
          type="text"
          ref={inputScheduleRef}
          placeholder="일정을 추가하세요."
        />
        <button type="submit">+</button>
      </form>
    </>
  );
}