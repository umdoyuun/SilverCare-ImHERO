import "./Toggle.css";

export default function StatusToggle({
  name,
  imgSrc,
  altSrc,
  statusLevel,
  status,
  symbol,
}) {
  return (
    <div id="status-toggle">
      <div id="status-toggle-box">
        <div className={statusLevel}>
          <img src={imgSrc} alt={altSrc} />
        </div>
        <div className="toggle-info">
          <p className="toggle-name">{name}</p>
          <p className={`${statusLevel}-status`}>
            {status + " "}
            <span style={{ fontSize: "0.7rem" }}>{symbol}</span>
          </p>
        </div>
      </div>
    </div>
  );
}
