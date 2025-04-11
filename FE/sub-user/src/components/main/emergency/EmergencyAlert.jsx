export default function EmergencyAlert({ emergencyAlert, onCheckAlert }) {
  const createdAtKST = new Date(emergencyAlert.created_at + "Z").toLocaleString(
    "ko-KR",
    {
      timeZone: "Asia/Seoul",
      year: "numeric",
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      hour12: true, // 24시간제
    }
  );

  const descriptions = emergencyAlert.description.split(",");

  return (
    <div
      id={
        emergencyAlert.is_read
          ? "emergency-alert-box-checked"
          : "emergency-alert-box"
      }
    >
      <div>
        <div className="title-container">
          <h2>{descriptions[0]}</h2>
          {descriptions.length > 1 && <p>{descriptions[1]}</p>}
        </div>
        <p className="date">{createdAtKST}</p>

        {/* 이미지 출력단 */}
        {emergencyAlert.image_url && (
          <div className="emergency-image-container">
            <img src={emergencyAlert.image_url} alt="temp" />
          </div>
        )}
      </div>
    </div>
  );
}
