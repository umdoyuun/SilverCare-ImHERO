import "./Mental.css";
import "../../spinner/Spinner.css";

import { useRef, useContext } from "react";

import { HealthContext } from "../../../store/healthStore";
import { UserProgressContext } from "../../../store/userProgressStore";

import Modal from "../../modal/Modal";

function handlePrint() {
  window.print(); // 현재 페이지의 내용을 출력
}

function DetailReport() {
  const healthStore = useContext(HealthContext);
  const detailReport = { ...healthStore.mentalHealthStatus };

  if (!detailReport) {
    return null;
  }

  const startDate = new Date(
    detailReport.analysis_period.start + "Z"
  ).toLocaleString("ko-KR", {
    timeZone: "Asia/Seoul",
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
  });

  const endDate = new Date(detailReport.analysis_period.end + "Z");
  endDate.setDate(endDate.getDate() - 1); // 하루 빼기

  const formattedDate = endDate.toLocaleString("ko-KR", {
    timeZone: "Asia/Seoul",
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
  });

  return (
    <div id="detail-report">
      <div className="detail-report-header">
        <h2>
          {startDate}부터 {formattedDate}까지의 정신 건강 분석 결과
        </h2>
        <h2 className={detailReport.overall_assessment.risk_level}>
          {detailReport.overall_assessment.total_score}
        </h2>
      </div>

      <div className="concerns-strengths">
        {detailReport.overall_assessment.strengths.length > 0 && (
          <div className="strengths">
            <h3>강점</h3>
            <ul>
              {detailReport.overall_assessment.strengths.map((strength) => {
                return <li key={strength}>{strength}</li>;
              })}
            </ul>
          </div>
        )}

        {detailReport.overall_assessment.concerns.length > 0 && (
          <div className="concerns">
            <h3>우려되는 점</h3>
            <ul>
              {detailReport.overall_assessment.concerns.map((concern) => {
                return <li key={concern}>{concern}</li>;
              })}
            </ul>
          </div>
        )}
      </div>

      <div className="analysis-summary">
        <div className="patterns">
          <div className="patterns-score">
            <h2>언어 사용 습관 분석</h2>
            <h3>{detailReport.language_patterns.score}</h3>
          </div>

          <div className="elem">
            <h3>자주 사용하는 단어와 의미 분석</h3>
            <p>{detailReport.language_patterns.word_choice}</p>
          </div>
          <div className="elem">
            <h3>문장 구조 분석</h3>
            <p>{detailReport.language_patterns.sentence_structure}</p>
          </div>
          <div className="elem">
            <h3>표현의 명확성 평가</h3>
            <p>{detailReport.language_patterns.expression_clarity}</p>
          </div>
        </div>

        <div className="patterns">
          <div className="patterns-score">
            <h2>맥락 이해 분석</h2>
            <h3>{detailReport.contextual_analysis.score}</h3>
          </div>

          <div className="elem">
            <h3>대화 주제 전환 능력</h3>
            <p>{detailReport.contextual_analysis.topic_flow}</p>
          </div>
          <div className="elem">
            <h3>대화 맥락 이해도</h3>
            <p>{detailReport.contextual_analysis.context_understanding}</p>
          </div>
        </div>

        <div className="patterns">
          <div className="patterns-score">
            <h2>인지 상태 분석</h2>
            <h3>{detailReport.cognitive_state.score}</h3>
          </div>

          <div className="elem">
            <h3>사고 명확성</h3>
            <p>{detailReport.cognitive_state.clarity}</p>
          </div>
          <div className="elem">
            <h3>대화 맥락 이해도</h3>
            <p>{detailReport.cognitive_state.reality_perception}</p>
          </div>
        </div>
      </div>

      {detailReport.recommendations.length > 0 && (
        <div className="recommendations">
          <h3>추천 사항</h3>
          <ul>
            {detailReport.recommendations.map((recommendation) => {
              return <li key={recommendation}>{recommendation}</li>;
            })}
          </ul>
        </div>
      )}

      <div className="report-control">
        <button className="print" onClick={handlePrint}>
          출력
        </button>
        <button
          className="close"
          onClick={healthStore.handleCloseMentalHealthReport}
        >
          닫기
        </button>
      </div>
    </div>
  );
}

export default function MentalHealthModal() {
  const healthStore = useContext(HealthContext);
  const userProgressStore = useContext(UserProgressContext);

  const startDateRef = useRef(null);
  const endDateRef = useRef(null);

  async function handleGetMentalHealthCustomReport() {
    const startDate = startDateRef.current.value;
    const endDate = endDateRef.current.value;

    if (!startDate || !endDate) {
      alert("날짜를 모두 입력해주세요!");
      return;
    }

    // 원하는 형식으로 변환
    const startDateTime = `${startDate}T00:00:00`;
    const endDateTime = `${endDate}T23:59:59`;

    try {
      const response = await healthStore.handleGetMentalHealthPeriodStatus(
        startDateTime,
        endDateTime
      );

      if (!response.success) {
        alert("정신 건강 상태 분석에 실패했습니다.");
      }
    } catch (error) {
      console.error(error);
    }
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "mental-health-report"}
      onClose={
        userProgressStore.modalProgress === "mental-health-report"
          ? healthStore.handleCloseMentalHealthReport
          : null
      }
    >
      <div id="mental-health-report">
        <div className="mental-health-report-header">
          <div className="period-report-control">
            <input
              type="date"
              id="start-date"
              name="start_date"
              ref={startDateRef}
              required
            />
            <input
              type="date"
              id="end-date"
              name="end_date"
              ref={endDateRef}
              required
            />
            <button onClick={handleGetMentalHealthCustomReport}>
              기간 분석 의뢰
            </button>
          </div>
        </div>
        <button onClick={healthStore.handleGetMentalHealthDailyStatus}>
          하루치 분석 의뢰
        </button>
      </div>

      {healthStore.loading && (
        <div className="loading-overlay">
          <div className="loader"></div>
        </div>
      )}

      {Object.keys(healthStore.mentalHealthStatus).length === 0 ? (
        <div className="order-report">
          <h2>감정을 의뢰해 보세요.</h2>
          <div className="report-control">
            <button
              className="close"
              onClick={healthStore.handleCloseMentalHealthReport}
            >
              닫기
            </button>
          </div>
        </div>
      ) : (
        <DetailReport />
      )}
    </Modal>
  );
}
