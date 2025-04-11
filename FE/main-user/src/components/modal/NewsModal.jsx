import React from "react";
import { useContext } from "react";
import { StoreContext } from "../../store/store.jsx";
import News from "../news/News";
import "./Modal.css";

export default function NewsModal({ title, onCloseConfirm }) {
  const store = useContext(StoreContext);

  const handleReply = () => {
    store.setOpenNewsState(false);
    setTimeout(() => {
      store.setOpenMessageState(true);
    }, 100);
  };

  return (
    <div id="modal-body">
      <div id="modal-bar">
        <h2>{title}</h2>
      </div>
      <News onReply={handleReply} />
    </div>
  );
}