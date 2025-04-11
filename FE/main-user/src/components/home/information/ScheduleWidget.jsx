import { useContext, useEffect, useState } from "react";
import { CalendarStoreContext } from "../../../store/calendarStore";
import "../Home.css"

export default function ScheduleWidget() {
  const { schedules } = useContext(CalendarStoreContext);
  const [today, setToday] = useState("");

  useEffect(() => {
    // 오늘 날짜 설정 (YYYY-MM-DD 형식)
    const currentDate = new Intl.DateTimeFormat("ko-KR", {
      year: "numeric",
      month: "2-digit",
      day: "2-digit",
      timeZone: "Asia/Seoul",
    }).format(new Date());
    setToday(currentDate);
  }, [schedules.schedules]);

  return (
      <div className="widget-schedule-date">{today}</div>
  );
}