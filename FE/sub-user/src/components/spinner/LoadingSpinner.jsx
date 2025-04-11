import "./Spinner.css";
import { createPortal } from "react-dom";
import nana from "../../assets/spinner/nana_spinner.gif";

export default function LoadingSpinner() {
  const spinnerRoot = document.getElementById("spinner"); // HTML의 `#spinner` 요소 찾기

  if (!spinnerRoot) return null; // `#spinner`가 없으면 렌더링 안 함

  return createPortal(
    <div className="loading-overlay">
      {/* <img src={nana} alt="Loading..." className="spinner-img" /> */}
      <div className="loader"></div>
    </div>,
    spinnerRoot
  );
}
