import "./Calendar.css";

// import CalendarStoreContextProvider from "../../../store/calendarStore";

import CalendarWidgetHeader from "./CalendarWidgetHeader";
import CalendarWidgetBody from "./CalendarWidgetBody";
import SelectedDate from "./SelectedDate";

export default function CalendarWidget() {
  return (
    // <CalendarStoreContextProvider>
    <div id="calendar-widget">
      <CalendarWidgetHeader />
      <CalendarWidgetBody />
      <SelectedDate />
    </div>

    // </CalendarStoreContextProvider>
  );
}
