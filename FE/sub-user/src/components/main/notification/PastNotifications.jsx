import "./Notification.css";

import { useRef, useState, useContext, useEffect } from "react";

import Modal from "../../modal/Modal.jsx";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";
import { EmergencyContext } from "../../../store/emergencyStore.jsx";

// import tempImg from "/lim.png";

export default function PastNotifications() {
  const userProgressStore = useContext(UserProgressContext);
  const emergencyStore = useContext(EmergencyContext);

  const [notifications, setNotifications] = useState([]);

  const startDateRef = useRef(null);
  const endDateRef = useRef(null);

  useEffect(() => {
    setNotifications([...emergencyStore.categorizedNotifications.info]);
  }, []);

  function handlePeriodAnalysis() {
    const startDate = startDateRef.current.value;
    const endDate = endDateRef.current.value;

    const filteredNotifications = emergencyStore.categorizedNotifications.info
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
      open={userProgressStore.modalProgress === "past-notifications"}
      onClose={
        userProgressStore.modalProgress === "past-notifications"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="past-notifications-title">
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

      <div id="past-notifications">
        <div id="home-notifications">
          {notifications.map((notification) => {
            // UTC+9 변환
            const createdAtKST = new Date(
              notification.created_at + "Z"
            ).toLocaleString("ko-KR", {
              timeZone: "Asia/Seoul",
              year: "numeric",
              month: "2-digit",
              day: "2-digit",
              hour: "2-digit",
              minute: "2-digit",
              hour12: true, // 24시간제
            });

            return (
              <div
                key={notification.index}
                className={
                  notification.is_read
                    ? "home-notification-checked"
                    : "home-notification"
                }
              >
                <div className="home-notification-icon-1">
                  <img src="" alt="" />
                </div>
                <div className="home-notification-content">
                  <div className="home-notification-description">
                    {notification.description}
                  </div>
                  <div className="home-notification-date">{createdAtKST}</div>
                </div>
              </div>
            );
          })}
        </div>
        <div className="emergency-alert-modal-actions">
          <button onClick={userProgressStore.handleCloseModal}>닫기</button>
        </div>
      </div>
    </Modal>
  );
}
