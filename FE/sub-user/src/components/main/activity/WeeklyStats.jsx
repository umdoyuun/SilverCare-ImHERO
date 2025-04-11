import WeeklyChart from "./WeeklyChart.jsx"

import WeeklySummary from "./WeeklySummary"

export default function WeeklyStats() {
  return (
    <div id="weekly-stats-container">
      {/* <div id="summary-container">
        <h2>지난 7일 요약</h2>
        <div>
          <WeeklySummary />
        </div>
      </div> */}
      <div id="weekly-chart-container">
        <WeeklyChart />
      </div>
    </div>
  )
}
