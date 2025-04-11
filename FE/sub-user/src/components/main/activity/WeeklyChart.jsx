import "./Activity.css"
import React, { useState, useEffect, useContext } from "react"
import { BarChart, Bar, XAxis, Tooltip, ResponsiveContainer } from "recharts"
import { HealthContext } from "../../../store/healthStore.jsx"
import { UserProgressContext } from "../../../store/userProgressStore.jsx"

const ActivityChart = () => {
  const healthStore = useContext(HealthContext)

  const weeklyData = healthStore.weeklyData

  return (
    <div id="activity-chart">
      <ResponsiveContainer>
        <BarChart data={weeklyData} padding={{ top: 10, right: 20, bottom: 15, left: 20 }}>
          <XAxis dataKey="name" />
          <Tooltip />
          <Bar dataKey="value" fill="#8884d8" />
        </BarChart>
      </ResponsiveContainer>
    </div>
  )
}

export default ActivityChart
