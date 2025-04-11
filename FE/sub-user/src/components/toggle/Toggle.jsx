import "./Toggle.css";

export default function Toggle({
  name,
  identifier,
  status,
  onClickToggle,
  imgSrc,
  altSrc,
}) {
  return (
    <div id="status-toggle">
      <div id="status-toggle-box">
        <div className={status}>
          <button
            className="toggle-button"
            // onClick={() => onClickToggle(identifier)}
          >
            <img src={imgSrc} alt={altSrc} />
          </button>
        </div>
        <div className="toggle-info">
          <p className="toggle-name">{name}</p>
          <p className={`${status}-status`}>
            {status === "good" ? "켜짐" : "꺼짐"}
          </p>
        </div>
      </div>
    </div>
  );
}
