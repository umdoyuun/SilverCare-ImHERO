import "./Home.css";

export default function Icon({ type, state, imgSrc, altSrc, onClickIcon, children, clicked, disabled }) {
  return (
    <div 
      id={type} 
      className={`clicked ? "clicked" : ""}  ${disabled ? "disable" : ""}`}
      onClick={onClickIcon}
      style={{ cursor: "none" }}
    >
      <img
        src={imgSrc}
        alt={altSrc}
        className={!state ? "disable-img" : "enable-img"}
      />
      {type === "icon" && <p className="icon-name">{children}</p>}
    </div>
  );
}