import "./Modal.css";

import { useRef, useEffect, useContext } from "react";
import { createPortal } from "react-dom";

import { EmergencyContext } from "../../store/emergencyStore";

export default function NotiModal({ children, open, onClose, className = "" }) {
  const emergencyStore = useContext(EmergencyContext);
  const dialog = useRef();

  useEffect(() => {
    const modal = dialog.current;

    if (open) {
      modal.show();
    }

    return () => modal.close();
  }, [open]);

  return createPortal(
    <dialog ref={dialog} className={`noti-modal`} onClose={onClose}>
      {children}
    </dialog>,
    document.getElementById("new-noti-modal")
  );
}
