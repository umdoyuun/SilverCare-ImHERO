import { useState } from "react";
import CalendarHeader from "./CalendarHeader.jsx";
import CalendarBody from "./CalendarBody.jsx";
import ScheduleModal from "../modal/ScheduleModal.jsx"; 
import "./Calendar.css";

export default function Calendar() {
  const [isModalOpen, setIsModalOpen] = useState(false);

  return (
    <div id="calendar">
      <CalendarHeader />
      <CalendarBody openModal={() => setIsModalOpen(true)} />
      <ScheduleModal isOpen={isModalOpen} onClose={() => setIsModalOpen(false)} />
    </div>
  );
}
