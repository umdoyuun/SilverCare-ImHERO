export default function StatusIcon({ imgSrc, altSrc, status, onClickIcon, className }) {
  return (
    <button 
      onClick={onClickIcon} 
      className={`${status ? "selected" : ""} ${className || ""}`.trim()}>
      <img src={imgSrc} alt={altSrc} />
    </button>
  );
}
