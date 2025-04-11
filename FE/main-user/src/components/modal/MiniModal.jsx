import React, { useContext } from "react";
import "./MiniModal.css";
import { StoreContext } from "../../store/store";

export default function MiniModal() {
  const { miniModalMessage } = useContext(StoreContext);

  if (!miniModalMessage) return null;

  return (
    <div className="mini-modal">
      <p>{miniModalMessage}</p>
    </div>
  );
}