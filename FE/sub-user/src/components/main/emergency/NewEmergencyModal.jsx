import "./Emergency.css";

import { useContext } from "react";

import Modal from "../../modal/Modal.jsx";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";
import { EmergencyContext } from "../../../store/emergencyStore.jsx";

// import tempImg from "/lim.png";

export default function NewEmergencyModal() {
  const userProgressStore = useContext(UserProgressContext);
  const emergencyStore = useContext(EmergencyContext);

  function handleCall(index) {
    emergencyStore.handleReadNotification(index);
    window.location.href = "tel:01012345678";
  }

  function handleMessage(index) {
    emergencyStore.handleReadNotification(index);
    window.location.href = "/message";
  }

  return (
    <Modal
      className="emergency-log-modal"
      open={emergencyStore.newCritNotifications.length > 0}
      // onClose={
      //   userProgressStore.modalProgress === "emergency-alert-log"
      //     ? userProgressStore.handleCloseModal
      //     : null
      // }
    >
      <h2>긴급 상황 발생</h2>

      <div id="emergency-alert-modal-box">
        {emergencyStore.newCritNotifications.slice().map((emergencyAlert) => {
          const createdAtKST = new Date(
            emergencyAlert.created_at + "Z"
          ).toLocaleString("ko-KR", {
            timeZone: "Asia/Seoul",
            month: "2-digit",
            day: "2-digit",
            hour: "2-digit",
            minute: "2-digit",
            hour12: true,
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
                    emergencyAlert.is_read ? "common" : "no-answer-title"
                  }
                >
                  {descriptions[0]}
                </h1>
                {descriptions.length > 1 && <p>{descriptions[1]}</p>}
              </div>
              <p className="date">{createdAtKST}</p>

              {/* 이미지 출력단 */}

              {emergencyAlert.image_url && (
                <div>
                  <img src={emergencyAlert.image_url} alt="temp" />
                </div>
              )}
              <div className="widget-button-container">
                {!emergencyAlert.is_read && (
                  <div className="widget-button-container">
                    <button
                      className="report"
                      onClick={() => handleMessage(emergencyAlert.index)}
                    >
                      메시지 전송
                    </button>
                    <button
                      className="call"
                      onClick={() => handleCall(emergencyAlert.index)}
                    >
                      전화 연결
                    </button>
                  </div>
                )}
                {emergencyAlert.is_read && (
                  <div className="widget-button-container">
                    <button
                      className="call"
                      onClick={() =>
                        emergencyStore.handleReadNotification(
                          emergencyAlert.index
                        )
                      }
                    >
                      전화 연결
                    </button>
                    <button
                      className="close"
                      onClick={() =>
                        emergencyStore.handleReadNotification(
                          emergencyAlert.index
                        )
                      }
                    >
                      닫기
                    </button>
                  </div>
                )}
              </div>
            </div>
          );
        })}
      </div>
    </Modal>
  );
}
