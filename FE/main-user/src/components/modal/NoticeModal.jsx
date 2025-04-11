import React from "react";
import { useContext } from "react";
import { StoreContext } from "../../store/store.jsx";
import Notice from "../notice/Notice";
import "./Modal.css";

export default function NoticeModal({ title, onCloseConfirm }) {
  const store = useContext(StoreContext);

  const handleReply = () => {
    store.setOpenNotificationState(false);
    setTimeout(() => {
      store.setOpenMessageState(true);
    }, 100);
  };

  return (
    <div id="modal-body">
      <div id="modal-bar">
        <h2>{title}</h2>
      </div>
      <Notice onReply={handleReply} />
      <button onClick={onCloseConfirm} className="button">
         닫기
      </button>
    </div>
  );
}