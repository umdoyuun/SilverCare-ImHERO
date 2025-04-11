import "./Notification.css";

import { useContext } from "react";

import { UserProgressContext } from "../../../store/userProgressStore";
import { EmergencyContext } from "../../../store/emergencyStore";

export default function PublicEmergencyNotifications() {
  const userProgressStore = useContext(UserProgressContext);
  const emergencyStore = useContext(EmergencyContext);

  function handleReadNotification(index) {
    emergencyStore.handleReadNotification(index);
  }

  return (
    <div id="public-emergency-notifications-container">
      <div id="public-emergency-notifications-header">
        <h3>공공재난 문자</h3>
      </div>
      <div id="public-emergency-notifications">
        {emergencyStore.categorizedNotifications.warn.length > 0 &&
          emergencyStore.categorizedNotifications.warn
            .slice()
            // .filter((notification) => !notification.is_read)
            .map((notification) => {
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
                <button
                  className="public-emergency-notification-click-btn"
                  key={notification.index}
                  onClick={() => handleReadNotification(notification.index)}
                >
                  <div
                    className={
                      notification.is_read
                        ? "public-emergency-notification-checked"
                        : "public-emergency-notification-notification"
                    }
                    key={notification.index}
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
                      {/* <p className="public-emergency-notification-loc">
                      {notification.description.RCPTN_RGN_NM}
                    </p> */}
                      <p className="public-emergency-notification-date">
                        {createdAtKST}
                      </p>
                    </div>
                  </div>
                </button>
              );
            })}
        {emergencyStore.categorizedNotifications.warn.length == 0 && (
          <h3>기록된 재난 문자가 없습니다.</h3>
        )}
      </div>
      <div className="public-emergency-notification-btn">
        <button
          onClick={() =>
            userProgressStore.handleOpenModal("past-public-emergency")
          }
        >
          지난 재난 문자 보기
        </button>
      </div>
    </div>
  );
}
