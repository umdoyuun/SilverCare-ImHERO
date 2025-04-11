import "./Emergency.css";

export default function EmergencyWidgetAlert({ notification }) {
  const createdAtKST = new Date(notification.created_at + "Z").toLocaleString(
    "ko-KR",
    {
      timeZone: "Asia/Seoul",
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      hour12: true,
    }
  );

  let parsedDescription;
  try {
    parsedDescription = JSON.parse(notification.description);
  } catch (e) {
    parsedDescription = notification.description; // JSON 파싱 실패 시 원본 유지
  }

  return (
    <div key={notification.index} id="notification">
      {notification.notification_grade === "info" && (
        <div className="notification-btn">
          <div className="home-notification-content">
            <div className="home-notification-description">
              {parsedDescription}
            </div>
            <p className="home-notification-date">{createdAtKST}</p>
          </div>
        </div>
      )}
      {notification.notification_grade === "warn" && (
        <div className="notification-btn-warn">
          <div className="home-notification-content">
            <div className="home-notification-content-header">
              <h2 className="home-notification-content-emergency">
                {parsedDescription.DST_SE_NM}
              </h2>
            </div>
            <p>{parsedDescription.EMRG_STEP_NM}</p>

            <div className="new-home-notification-description">
              {parsedDescription.MSG_CN}
            </div>

            <p className="home-notification-date">{createdAtKST}</p>
          </div>
        </div>
      )}
      {notification.notification_grade === "crit" && (
        <div className="notification-btn-crit">
          <div className="home-notification-content">
            <div className="home-notification-content-header">
              <h2 className="home-notification-content-emergency">
                {parsedDescription.split(",")[0]}
              </h2>
              {parsedDescription.split(",").length > 1 && (
                <span>{parsedDescription.split(",")[1]}</span>
              )}
            </div>
            {notification.image_url && (
              <div>
                <img
                  className="crit-image"
                  src={notification.image_url}
                  alt=""
                />
              </div>
            )}
            <p className="home-notification-date">{createdAtKST}</p>
          </div>
        </div>
      )}
    </div>
  );
}
