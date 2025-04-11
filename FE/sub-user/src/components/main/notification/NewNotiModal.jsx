import "./Notification.css";

import { useContext } from "react";
import { EmergencyContext } from "../../../store/emergencyStore.jsx";

import NotiModal from "../../modal/NotiModal.jsx";

export default function NewNotiModal() {
  const emergencyStore = useContext(EmergencyContext);

  function handleReadNotification(index) {
    emergencyStore.handleReadNotification(index);
  }

  return (
    <NotiModal
      open={emergencyStore.newNotifications.length > 0}
      onClose={
        emergencyStore.newNotifications.length > 0
          ? emergencyStore.handleClearNewNotifications
          : null
      }
    >
      <div id="modal-notifications-actions">
        <h3>미확인 알림</h3>
        <button onClick={emergencyStore.handleCheckAllAlert}>전체 읽음</button>
      </div>
      <div id="modal-notifications">
        <div id="home-notifications">
          {emergencyStore.newNotifications.slice().map((notification) => {
            // UTC+9 변환
            const createdAtKST = new Date(
              notification.created_at + "Z"
            ).toLocaleString("ko-KR", {
              timeZone: "Asia/Seoul",
              month: "2-digit",
              day: "2-digit",
              hour: "2-digit",
              minute: "2-digit",
              hour12: true,
            });

            let parsedDescription;
            try {
              parsedDescription = JSON.parse(notification.description);
            } catch (e) {
              parsedDescription = notification.description; // JSON 파싱 실패 시 원본 유지
            }

            return (
              <div
                key={notification.index} // 리스트에는 key 필수!
                className="home-notification"
              >
                {notification.notification_grade === "info" && (
                  <button
                    className="new-notification-btn"
                    onClick={() => handleReadNotification(notification.index)}
                  >
                    <div className="home-notification-icon-1">
                      <img src="" alt="" />
                    </div>
                    <div className="home-notification-content">
                      <div className="home-notification-description">
                        {parsedDescription}
                      </div>
                      <div className="home-notification-date">
                        {createdAtKST}
                      </div>
                    </div>
                  </button>
                )}
                {notification.notification_grade === "warn" && (
                  <button
                    className="new-notification-btn"
                    onClick={() => handleReadNotification(notification.index)}
                  >
                    <div className="home-notification-icon-2">
                      <img src="" alt="" />
                    </div>
                    <div className="home-notification-content">
                      <div className="home-notification-content-header">
                        <h2>{parsedDescription.DST_SE_NM}</h2>
                      </div>

                      <div className="new-home-notification-description">
                        <p>{parsedDescription.EMRG_STEP_NM}</p>
                        {parsedDescription.MSG_CN}
                      </div>

                      <div className="home-notification-date">
                        {createdAtKST}
                      </div>
                    </div>
                  </button>
                )}
              </div>
            );
          })}
        </div>
      </div>
    </NotiModal>
  );
}
