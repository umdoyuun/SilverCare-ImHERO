import "./Accounts.css";

import { useRef, useState, useContext } from "react";
import { useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";

export default function DeleteFamily() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const [passwordIsInvalid, setPasswordIsInvalid] = useState(false);
  const [isCapsLockOn, setIsCapsLockOn] = useState(false);

  const inputPassword = useRef("");

  const handleKeyDown = (event) => {
    if (event.getModifierState("CapsLock")) {
      setIsCapsLockOn(true);
    } else {
      setIsCapsLockOn(false);
    }
  };

  const handleKeyUp = (event) => {
    if (!event.getModifierState("CapsLock")) {
      setIsCapsLockOn(false);
    }
  };

  // 가족 삭제 로직
  async function handleDeleteFamily() {
    // 비밀번호 유효성 검사
    const inputIsInvalid = inputPassword.current.value.length < 8;
    if (inputIsInvalid) {
      setPasswordIsInvalid(true);
      return;
    }

    userProgressStore.handleCloseModal();

    try {
      const result = await userProgressStore.handleDeleteFamilyInfo(
        inputPassword.current.value
      );

      if (result.success === true) {
        // 가족 삭제 성공
        alert("가족 모임 삭제 성공");

        inputPassword.current.value = "";

        navigate("/accounts");
      } else {
        userProgressStore.handleOpenModal("delete-family");

        console.error("가족 모임 삭제 실패:", result.error);
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
      alert("요청 처리 중 문제가 발생했습니다. 다시 시도해주세요.");
    }
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "delete-family"}
      onClose={
        userProgressStore.modalProgress === "delete-family"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="signout-form">
        <div className="signup-header">
          <h2>가족 모임 삭제</h2>
          <button type="button" onClick={userProgressStore.handleCloseModal}>
            ⨉
          </button>
        </div>
        <div className="signout-control">
          <label htmlFor="password">비밀번호</label>
          <input
            type="password"
            ref={inputPassword}
            onKeyDown={handleKeyDown}
            onKeyUp={handleKeyUp}
          />
          <div className="login-control-error">
            {passwordIsInvalid && <p>⚠️ 비밀번호는 8자 이상입니다.</p>}
            {isCapsLockOn && <p>⚠️ Caps Lock이 켜져 있습니다!</p>}
          </div>
          <button className="logout-btn" onClick={handleDeleteFamily}>
            모임 삭제
          </button>
        </div>
      </div>
    </Modal>
  );
}
