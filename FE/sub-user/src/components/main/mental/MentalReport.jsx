import "./Mental.css"

import { useContext } from "react"

import { HealthContext } from "../../../store/healthStore"

import MentalReportDetail from "./MentalReportDetail"

export default function MentalReport() {
  const healthStore = useContext(HealthContext)

  const report = healthStore.mentalStatus && healthStore.mentalStatus.length > 0 ? healthStore.mentalStatus[0] : null

  const mainUserName = "박순자123"

  return (
    <div id="mental-reports">
      <div className="mental-report-header">
        <h3>{report ? report.reported_at.slice(0, 10) : null} 감정 상태 보고서</h3>
        <button onClick={healthStore.handleGetOneDayMentalReport}>지난 하루 감정 보고서 요청</button>
      </div>
      {healthStore.mentalStatus.length === 0 && (
        <div className="no-mental-report">
          <p>기록된 감정 정보가 없습니다.</p>
        </div>
      )}
      {healthStore.mentalStatus.length > 0 && (
        <div id="mental-report">
          <div className="mental-report-overall">
            <h3 className="mental-report-subtitle">전반적 통찰 상태</h3>
            <ol>
              <li>{report ? report.description.overall_emotional_state : null}</li>
            </ol>
          </div>
          <div className="mental-report-recommendation">
            <h3 className="mental-report-subtitle">권고 사항</h3>
            <ol>
              {report &&
                report.description.recommendations.map((recommendation) => {
                  return <li key={recommendation}>{recommendation}</li>
                })}
            </ol>
          </div>

          <button className="detail-report-btn" onClick={() => healthStore.handleShowDetailReport()}>
            상세 보고서 보기
          </button>
        </div>
      )}
      <MentalReportDetail />
    </div>
  )
}
