import "./Accounts.css";

import { useRef, useState, useContext } from "react";
import { useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";

export default function UpdateMember() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const [nameIsInvalid, setNameIsInvalid] = useState(false);

  const inputName = useRef("");

  async function handleUpdateMember(event) {
    event.preventDefault();

    // 닉네임 유효성 검사
    const invalid =
      inputName.current.value.length < 2 || inputName.current.value.length > 32;
    if (invalid) {
      setNameIsInvalid(true);
      return;
    } else {
      setNameIsInvalid(false);
    }

    userProgressStore.handleCloseModal();

    try {
      const result = await userProgressStore.handleUpdateMember(
        inputName.current.value
      );
      if (result.success === true) {
        // 닉네임 수정 성공
        alert("닉네임 수정 성공");
        inputName.current.value = "";
        navigate("/accounts");
      } else {
        userProgressStore.handleOpenModal("update-member-user-info");
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
    }
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "update-member-user-info"}
      onClose={
        userProgressStore.modalProgress === "update-member-user-info"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="signup-form">
        <div className="signup-header">
          <h2>닉네임 정보 수정</h2>
          <button type="button" onClick={userProgressStore.handleCloseModal}>
            ⨉
          </button>
        </div>
        <p className="signup-control">
          <label htmlFor="text">닉네임</label>
          <input type="text" ref={inputName} />
          {nameIsInvalid && (
            <div className="signup-control-error">
              <p>닉네임은 2글자 이상 32글자 이하여야 합니다.</p>
            </div>
          )}
          <button className="signup-btn" onClick={handleUpdateMember}>
            수정
          </button>
        </p>
      </div>
    </Modal>
  );
}
