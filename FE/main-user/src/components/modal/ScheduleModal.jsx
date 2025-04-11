import { useContext } from "react";
import { CalendarStoreContext } from "../../store/calendarStore";
import "./MiniModal.css";

export default function ScheduleModal({ isOpen, onClose }) {
  const { selectedDate, schedules } = useContext(CalendarStoreContext);

  if (!isOpen) return null;

  return (
    <div className="modal-overlay">
      <div className="modal-content">
        <h2>{selectedDate.date}의 일정</h2>
        <ul>
          {schedules.schedules[selectedDate.date]?.map((schedule, index) => (
            <li key={index}>{schedule}</li>
          )) || <li>해당 날짜에는 일정이 없습니다.</li>}
        </ul>
        
        <button className="mini-button" onClick={onClose}>X</button>
      </div>
    </div>
  );
}