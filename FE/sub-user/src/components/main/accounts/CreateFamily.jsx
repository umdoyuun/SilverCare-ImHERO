import "./Accounts.css";

import { useRef, useState, useContext } from "react";
import { useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";

export default function CreateFamily() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const [nameIsInvalid, setNameIsInvalid] = useState(false);

  const inputName = useRef("");

  async function handleCreateFamily(event) {
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

    // 입력받은 데이터 객체화
    const payload = {
      main_user: userProgressStore.loginUserInfo.userInfo.id,
      family_name: inputName.current.value,
    };

    userProgressStore.handleCloseModal();

    try {
      const result = await userProgressStore.handleCreateFamily(payload);
      if (result.success === true) {
        // 가족 생성 성공
        alert("가족 모임 생성 성공");

        inputName.current.value = "";

        navigate("/accounts");
      } else {
        userProgressStore.handleOpenModal("create-family-user-info");
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
      alert("요청 처리 중 문제가 발생했습니다. 다시 시도해주세요.");
    }
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "create-family-user-info"}
      onClose={
        userProgressStore.modalProgress === "create-family-user-info"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="signup-form">
        <div className="signup-header">
          <h2>가족 모임 생성</h2>
          <button type="button" onClick={userProgressStore.handleCloseModal}>
            ⨉
          </button>
        </div>
        <p className="signup-control">
          <label htmlFor="text">가족 모임 이름</label>
          <input type="text" ref={inputName} />
          {nameIsInvalid && (
            <div className="signup-control-error">
              <p>가족 모임 이름은 2글자 이상 32글자 이하여야 합니다.</p>
            </div>
          )}
          <button className="signup-btn" onClick={handleCreateFamily}>
            모임 생성
          </button>
        </p>
      </div>
    </Modal>
  );
}
