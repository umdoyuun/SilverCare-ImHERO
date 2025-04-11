import "./Accounts.css";

import { useRef, useState, useContext } from "react";
import { useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";

export default function UpdateFamily() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const [nameIsInvalid, setNameIsInvalid] = useState(false);

  const inputName = useRef("");

  async function handleUpdateFamily(event) {
    event.preventDefault();

    // 가족 이름 유효성 검사
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
      const result = await userProgressStore.handleUpdateFamilyInfo(
        inputName.current.value
      );
      if (result.success === true) {
        // 가족 정보 수정 성공
        alert("가족 모임 정보 수정 성공");

        inputName.current.value = "";

        navigate("/accounts");
      } else {
        userProgressStore.handleOpenModal("update-family-user-info");
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
    }
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "update-family-user-info"}
      onClose={
        userProgressStore.modalProgress === "update-family-user-info"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="signup-form">
        <div className="signup-header">
          <h2>가족 모임 정보 수정</h2>
          <button type="button" onClick={userProgressStore.handleCloseModal}>
            ⨉
          </button>
        </div>
        <p className="signup-control">
          <label htmlFor="text">수정할 가족 모임 이름</label>
          <input type="text" ref={inputName} />
          {nameIsInvalid && (
            <div className="signup-control-error">
              <p>가족 모임 이름은 2글자 이상 32이하여야 합니다.</p>
            </div>
          )}
          <button className="signup-btn" onClick={handleUpdateFamily}>
            수정
          </button>
        </p>
      </div>
    </Modal>
  );
}
