import "./Activity.css"

import { useContext } from "react"

import { HealthContext } from "../../../store/healthStore.jsx"

export default function ActivityToggle({ name, imgSrc, altSrc, status, isActive }) {
  const healthStore = useContext(HealthContext)

  function handleSwitchChart() {
    healthStore.handleChangeSelectedToggle(altSrc)
  }

  return (
    <button id={isActive ? "active-activity-toggle" : "activity-toggle"} onClick={handleSwitchChart}>
      <div id="activity-toggle-box">
        <div className={altSrc === "mental" ? "toggle-mental" : "toggle-activity"}>
          <img src={imgSrc} alt={altSrc} />
        </div>
        <div className="toggle-info">
          <p className="toggle-name">{name}</p>
          <p className={status < 70 ? "toggle-status-bad" : "toggle-status-good"}>{status}</p>
        </div>
      </div>
    </button>
  )
}
