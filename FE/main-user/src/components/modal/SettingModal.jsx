import "./Modal.css";
import "../settings/Settings";
import Backgrounds from "../settings/Backgrounds";"../settings/Backgrounds";
import Settings from "../settings/Settings";

export default function SettingModal({ title, onCloseConfirm }) {
  return (
    <div id="modal-body">
      <div id="modal-bar">
        <h2>{title}</h2>
      </div>
      <div id="setting-box">
        <Settings
            type="box"
        >
            <div id="setting-name">
              배경화면 변경
            </div>
            <Backgrounds/>
        </Settings>
      </div>
      <button onClick={onCloseConfirm} className="button">
         닫기
      </button>
    </div>
  );
}
