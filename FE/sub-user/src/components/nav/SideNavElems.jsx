import { useNavigate } from "react-router-dom"

export default function SideNavElems({ imgSrc, altSrc, identifier, text, activeIdentifier, onClickElem }) {
  const navigate = useNavigate()

  return (
    <li className="side-nav-li">
      <button
        className={activeIdentifier === altSrc ? "side-nav-elem-active" : "side-nav-elem"}
        onClick={() => {
          onClickElem(altSrc)
          if (altSrc === "home") {
            navigate("/")
          } else {
            navigate(`/${altSrc}`)
          }
        }}
      >
        <div className="side-nav-icon">
          <img src={imgSrc} alt={altSrc} />
        </div>
        <div className="side-nav-text">{text}</div>
      </button>
    </li>
  )
}
