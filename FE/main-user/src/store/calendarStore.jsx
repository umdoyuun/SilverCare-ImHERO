import { useState, createContext, useEffect } from "react";
import  useCalendar from "../hooks/useCalendar";

export const CalendarStoreContext = createContext({
  currentDate: {
    year: "",
    month: "",
    day: "",
  },
  daysInMonth: [],
  dispatch: {
    handlePrevYear: () => {},
    handleNextYear: () => {},
    handlePrevMonth: () => {},
    handleNextMonth: () => {},
  },
  selectedDate: {
    date: "",
    selectedDate: () => {},
  },
  schedules: {
    schedules: [],
    addSchedule: () => {},
  },
  modal: {
    isModalOpen: false,
    openModal: () => {},
    closeModal: () => {},
  }
});

export default function CalendarStoreContextProvider({ children }) {
  const context = useCalendar();

  const [selectedDate, setSelectedDate] = useState("");
  const selectDate = (date) => {
    setSelectedDate(date);
    setIsModalOpen(true);
  }

  // 상태 관리
  const [schedules, setSchedules] = useState(() => {
    // localStorage에서 초기 데이터 로드
    const storedSchedules = localStorage.getItem("schedules");
    return storedSchedules ? JSON.parse(storedSchedules) : {};
  });

  // 상태 변경 시 localStorage에 저장
  useEffect(() => {
    localStorage.setItem("schedules", JSON.stringify(schedules));
  }, [schedules]);

  // 스케줄 추가 함수
  const addSchedule = (date, schedule) => {
    setSchedules((prevSchedules) => {
      const updatedSchedules = {
        ...prevSchedules,
        [date]: prevSchedules[date]
          ? [...prevSchedules[date], schedule]
          : [schedule],
      };
      return updatedSchedules;
    });
  };

  const [isModalOpen, setIsModalOpen] = useState(false);
  const openModal = () => setIsModalOpen(true);
  const closeModal = () => setIsModalOpen(false);

  const ctxValue = {
    ...context,
    selectedDate: {
      date: selectedDate,
      selectDate,
    },
    schedules: {
      schedules,
      addSchedule,
    },
    modal: {
      isModalOpen,
      openModal,
      closeModal,
    }
  };

  return (
    <CalendarStoreContext.Provider value={ctxValue}>
      {children}
    </CalendarStoreContext.Provider>
  );
}