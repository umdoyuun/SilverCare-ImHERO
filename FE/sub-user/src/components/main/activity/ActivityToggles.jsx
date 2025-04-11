import "./Activity.css"

import { useContext } from "react"

import { HealthContext } from "../../../store/healthStore.jsx"

import ActivityToggle from "./ActivityToggle.jsx"

import heartIcon from "../../../assets/icons/heart_plus.svg"
import walkIcon from "../../../assets/icons/walk.svg"

export default function ActivityToggles() {
  const healthStore = useContext(HealthContext)

  const mentalScore = healthStore.mentalStatus && healthStore.mentalStatus.length > 0 ? healthStore.mentalStatus[0].score : null

  const activityScore = healthStore.activityStatus && healthStore.activityStatus.length > 0 ? healthStore.activityStatus[0].score : null

  return (
    <div id="activity-toggle-group">
      <ActivityToggle name="활동" imgSrc={walkIcon} altSrc="activity" status={activityScore} isActive={healthStore.selectedToggle === "activity"} />
      <ActivityToggle name="정신 건강" imgSrc={heartIcon} altSrc="mental" status={mentalScore} isActive={healthStore.selectedToggle === "mental"} />
    </div>
  )
}
