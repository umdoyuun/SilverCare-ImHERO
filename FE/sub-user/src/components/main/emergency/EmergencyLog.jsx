import { useRef, useState, useEffect, useContext } from "react";

import Modal from "../../modal/Modal.jsx";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";
import { EmergencyContext } from "../../../store/emergencyStore.jsx";

// import tempImg from "/lim.png";

export default function EmergencyLog() {
  const userProgressStore = useContext(UserProgressContext);
  const emergencyStore = useContext(EmergencyContext);

  const [notifications, setNotifications] = useState([]);

  const startDateRef = useRef(null);
  const endDateRef = useRef(null);

  useEffect(() => {
    setNotifications([...emergencyStore.categorizedNotifications.crit]);
  }, []);

  function handlePeriodAnalysis() {
    const startDate = startDateRef.current.value;
    const endDate = endDateRef.current.value;

    const filteredNotifications = emergencyStore.categorizedNotifications.crit
      .slice()
      .filter((notification) => {
        const dateKST = new Date(notification.created_at);
        dateKST.setHours(dateKST.getHours() + 9);
        return (
          new Date(dateKST) >= new Date(startDate) &&
          new Date(dateKST) <= new Date(endDate)
        );
      });

    setNotifications(filteredNotifications);
  }

  return (
    <Modal
      className="emergency-log-modal"
      open={userProgressStore.modalProgress === "emergency-alert-log"}
      onClose={
        userProgressStore.modalProgress === "emergency-alert-log"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="emergency-alert-modal-title">
        <h2>지난 알림 기록</h2>
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
          <button onClick={handlePeriodAnalysis}>기간 설정</button>
        </div>
      </div>

      <div id="emergency-alert-modal-box">
        {notifications.map((emergencyAlert, index) => {
          const createdAtKST = new Date(
            emergencyAlert.created_at + "Z"
          ).toLocaleString("ko-KR", {
            timeZone: "Asia/Seoul",
            year: "numeric",
            month: "2-digit",
            day: "2-digit",
            hour: "2-digit",
            minute: "2-digit",
            hour12: true, // 24시간제
          });

          const descriptions = emergencyAlert.description.split(",");

          return (
            <div
              key={emergencyAlert.index}
              className={
                emergencyAlert.is_read ? "alert-box-check" : "alert-box"
              }
            >
              <div className="title-container">
                <h1
                  className={
                    emergencyAlert.is_check ? "common" : "no-answer-title"
                  }
                >
                  {descriptions[0]}
                </h1>
                {descriptions.length > 1 && <p>{descriptions[1]}</p>}
              </div>
              <p className="time">{createdAtKST}</p>

              {/* 이미지 출력단 */}
              {emergencyAlert.image_url && (
                <div>
                  <img src={emergencyAlert.image_url} alt="temp" />
                </div>
              )}
            </div>
          );
        })}
      </div>

      <div className="emergency-alert-modal-actions">
        <button onClick={userProgressStore.handleCloseModal}>닫기</button>
      </div>
    </Modal>
  );
}
