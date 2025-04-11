import "./Notification.css";

import { useRef, useState, useContext, useEffect } from "react";

import Modal from "../../modal/Modal.jsx";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";
import { EmergencyContext } from "../../../store/emergencyStore.jsx";

export default function PastPublicEmergency() {
  const userProgressStore = useContext(UserProgressContext);
  const emergencyStore = useContext(EmergencyContext);

  const [notifications, setNotifications] = useState([]);

  const startDateRef = useRef(null);
  const endDateRef = useRef(null);

  useEffect(() => {
    setNotifications([...emergencyStore.categorizedNotifications.warn]);
  }, []);

  function handlePeriodAnalysis() {
    const startDate = startDateRef.current.value;
    const endDate = endDateRef.current.value;

    const filteredNotifications = emergencyStore.categorizedNotifications.warn
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
      open={userProgressStore.modalProgress === "past-public-emergency"}
      onClose={
        userProgressStore.modalProgress === "past-public-emergency"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="past-notifications-title">
        <h2>지난 재난 문자 기록</h2>
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
        <div id="public-emergency-notifications">
          {notifications.map((notification) => {
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
                    ? "public-emergency-notification-checked"
                    : "public-emergency-notification"
                }
              >
                <div className="public-emergency-notification-header">
                  <h2>{notification.description.DST_SE_NM}</h2>
                  <p>{notification.description.EMRG_STEP_NM}</p>
                </div>
                <div className="public-emergency-notification-content">
                  <div className="public-emergency-notification-description">
                    {notification.description.MSG_CN}
                  </div>
                </div>
                <div className="public-emergency-notification-footer">
                  <p className="public-emergency-notification-loc">
                    {notification.description.RCPTN_RGN_NM}
                  </p>
                  <p className="public-emergency-notification-date">
                    {createdAtKST}
                  </p>
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
