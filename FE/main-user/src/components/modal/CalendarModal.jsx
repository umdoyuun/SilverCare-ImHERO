import React from "react"
import Calendar from "../calendar/Calendar"
import "./Modal.css"

import CalendarStoreContextProvider from "../../store/calendarStore"

export default function CalendarModal({ title, onCloseConfirm }) {
  return (
    <div id="modal-body-thin">
      <div id="modal-bar-thin">
        <h2>{title}</h2>
        <button onClick={onCloseConfirm} className="close-button">
          ✖
        </button>
      </div>
      <CalendarStoreContextProvider>
        <Calendar />
      </CalendarStoreContextProvider>
    </div>
  )
}
