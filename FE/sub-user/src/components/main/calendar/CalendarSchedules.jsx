import { useContext, useRef } from "react";

import { CalendarStoreContext } from "../../../store/calendarStore";
import { HealthContext } from "../../../store/healthStore";

import MentalReportDetail from "../mental/MentalReportDetail";

import activityImage from "../../../assets/icons/run.svg";
import mindfulnessImage from "../../../assets/icons/mindfulness.svg";
import thermostatImage from "../../../assets/feature/temp-hot-line.png";
import airImage from "../../../assets/feature/sparkling-line.png";
import heatImage from "../../../assets/feature/blaze-line.png";
import humidityImage from "../../../assets/feature/water-percent-line.png";

export default function CalendarSchedules() {
  const { selectedDate, schedules } = useContext(CalendarStoreContext);
  const { mentalStatus, handleShowDetailReport } = useContext(HealthContext);

  const healthData = schedules.schedules.health;
  const mentalData = schedules.schedules.mental;
  const homeStatusData = schedules.schedules.homeStatus;

  const inputScheduleRef = useRef(""); // ref 초기화

  function handleOpenSpecificMentalReport(targetIndex) {
    const copyMentalStatus = [...mentalStatus];

    const arrayIndex = copyMentalStatus.findIndex(
      (report) => report.index === targetIndex
    );

    if (arrayIndex !== -1) {
      // 선택된 감정 보고서 출력
      handleShowDetailReport(arrayIndex);
    } else {
      console.error("해당되는 감정 보고서를 찾을 수 없습니다.");
    }
  }

  function handleSubmitSchedule(event) {
    event.preventDefault();

    const newSchedule = inputScheduleRef.current.value; // 입력값 가져오기
    if (newSchedule.trim()) {
      schedules.addSchedule(selectedDate.date, newSchedule); // 새 일정 추가
      inputScheduleRef.current.value = ""; // 입력값 초기화
    }
  }

  return (
    <div className="calendar-schedule-container">
      <div className="calendar-schedule-date">
        <a>{selectedDate.date}</a>
      </div>

      {/* 날짜 별 집안 상태 평균 점수 표시 */}
      <div className="calendar-schedule-reports-box">
        <li className="calendar-day-status-schedules">
          <ul className="main-status-aver">
            <img
              className="calendar-widget-status-icon"
              src={thermostatImage}
              alt="temperature"
            />
            <div>
              {homeStatusData[selectedDate.date] ? (
                <>
                  {homeStatusData[selectedDate.date].temperature.toFixed(1)}
                  <span style={{ fontSize: "0.8em" }}> ℃</span>
                </>
              ) : (
                "-"
              )}
            </div>
          </ul>
          <ul className="main-status-aver">
            <img
              className="calendar-widget-status-icon"
              src={humidityImage}
              alt="humidity"
            />
            <div>
              {homeStatusData[selectedDate.date] ? (
                <>
                  {homeStatusData[selectedDate.date].humidity.toFixed(1)}
                  <span style={{ fontSize: "0.8rem" }}> %</span>
                </>
              ) : (
                "-"
              )}
            </div>
          </ul>
          <ul className="main-status-aver">
            <img
              className="calendar-widget-status-icon"
              src={airImage}
              alt="dust-level"
            />
            <div>
              {homeStatusData[selectedDate.date] ? (
                <>
                  {homeStatusData[selectedDate.date].dust_level.toFixed(1)}
                  <span style={{ fontSize: "0.8rem" }}> ㎍/㎥</span>
                </>
              ) : (
                "-"
              )}
            </div>
          </ul>
          <ul className="main-status-aver">
            <img
              className="calendar-widget-status-icon"
              src={heatImage}
              alt="ethanol"
            />
            <div>
              {homeStatusData[selectedDate.date] ? (
                <>
                  {homeStatusData[selectedDate.date].ethanol.toFixed(1)}
                  <span style={{ fontSize: "0.8rem" }}> %</span>
                </>
              ) : (
                "-"
              )}
            </div>
          </ul>
        </li>
      </div>

      <div className="calendar-schedule-results-box">
        <div className="calendar-health-reports">
          <div className="calendar-health-reports-header">
            <img src={activityImage} alt="activity" />
            {healthData[selectedDate.date] ? (
              <h3>평균: {healthData[selectedDate.date].averageScore}</h3>
            ) : (
              <h3>-</h3>
            )}
          </div>
          {/* 해당 날짜 일정 출력 */}
          <ul>
            {healthData[selectedDate.date] ? (
              healthData[selectedDate.date].records
                .slice() // 원본 배열 변경 방지
                .reverse()
                .map((record, index) => {
                  // UTC+9 변환
                  const reportedAtKST = new Date(
                    record.reported_at
                  ).toLocaleString("ko-KR", {
                    timeZone: "Asia/Seoul",
                    hour: "2-digit",
                    minute: "2-digit",
                    hour12: true, // 24시간제
                  });

                  return (
                    <li key={index}>
                      <button>
                        <div className="health-report">
                          <span
                            className={
                              record.is_critical
                                ? "critical-score"
                                : "health-score"
                            }
                          >
                            {record.score}
                          </span>
                          <div className="health-report-header">
                            <span>{reportedAtKST}</span>{" "}
                            {/* 변환된 시간 사용 */}
                            <p>{record.action}</p>
                          </div>
                        </div>
                      </button>
                    </li>
                  );
                })
            ) : (
              <div className="no-data">기록된 활동 정보가 없습니다.</div>
            )}
          </ul>
        </div>
        <div className="calendar-mental-reports">
          <div className="calendar-mental-reports-header">
            <img src={mindfulnessImage} alt="mental" />
            {mentalData[selectedDate.date] ? (
              <h3>평균: {mentalData[selectedDate.date].averageScore}</h3>
            ) : (
              <h3>-</h3>
            )}
          </div>
          <ul>
            {/* 해당 날짜 일정 출력력 */}
            {mentalData[selectedDate.date] ? (
              mentalData[selectedDate.date].records
                .slice()
                .reverse()
                .map((record, index) => {
                  // UTC+9 변환
                  const reportedAtKST = new Date(
                    record.reported_at
                  ).toLocaleString("ko-KR", {
                    timeZone: "Asia/Seoul",
                    hour: "2-digit",
                    minute: "2-digit",
                    hour12: true, // 24시간제
                  });

                  return (
                    <li key={index}>
                      <button
                        onClick={() =>
                          handleOpenSpecificMentalReport(record.index)
                        }
                      >
                        <div className="mental-report">
                          <span
                            className={
                              record.is_critical
                                ? "critical-score"
                                : "mental-score"
                            }
                          >
                            {record.score}
                          </span>
                          <div className="mental-report-header">
                            <span>{reportedAtKST}</span>
                            <span>
                              <p>
                                {record.description.overall_emotional_state}
                              </p>
                            </span>
                          </div>
                        </div>
                      </button>
                    </li>
                  );
                })
            ) : (
              <div className="no-data">가록된 정신 건강 정보가 없습니다.</div>
            )}
          </ul>
        </div>
      </div>
      <MentalReportDetail />

      {/* 일정 추가 input 그룹 */}
      {/* <form onSubmit={handleSubmitSchedule} className="calendar-form">
        <input
          type="text"
          ref={inputScheduleRef}
          placeholder="Add a schedule"
        />
        <button type="submit">+</button>
      </form> */}
    </div>
  );
}
