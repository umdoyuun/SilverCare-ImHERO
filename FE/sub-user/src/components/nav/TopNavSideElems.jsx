import { useNavigate } from "react-router-dom"

export default function TopNavSideElems({ imgSrc, altSrc, identifier, activeIdentifier, onClickElem }) {
  const navigate = useNavigate()

  return (
    <li className="top-side-bar-nav-li">
      <button
        className={activeIdentifier === altSrc ? "top-side-bar-nav-elem-active" : "top-side-bar-nav-elem"}
        onClick={() => {
          onClickElem(altSrc)
          if (altSrc === "home") {
            navigate("/")
          } else {
            navigate(`/${altSrc}`)
          }
        }}
      >
        <div className="top-side-bar-nav-icon">
          <img src={imgSrc} alt={altSrc} />
        </div>
        <div className="top-side-bar-nav-text">{identifier}</div>
      </button>
    </li>
  )
}
