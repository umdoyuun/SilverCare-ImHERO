import "./Accounts.css";

import { useRef, useState, useContext } from "react";
import { useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";

export default function CreateFamily() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const [familyIdChecked, setFamilyIdChecked] = useState(false);
  const [nameIsInvalid, setNameIsInvalid] = useState(false);

  const inputFamilyId = useRef("");
  const inputName = useRef("");

  // 가족 구성원 조회 및 정보 저장
  async function handleCheckFamily() {
    const familyId = inputFamilyId.current.value;

    if (!familyId) {
      alert("가족 모임 ID를 입력해주세요.");
      return;
    }

    if (familyId.includes("/")) {
      alert("가족 모임 ID에 사용 불가능한 문자가 사용되었습니다.");
      return;
    }

    try {
      const response = await fetch(
        `${userProgressStore.DEV_API_URL}/families/name/${familyId}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include",
        }
      );

      const resData = await response.json();

      if (response.ok) {
        if (resData.message === "Family retrieved successfully") {
          console.log("가족 구성원 조회 성공");

          // 검색된 가족 모임 이름 저장
          setFamilyIdChecked(resData.result);

          return { success: true, data: resData };
        }
      } else {
        // 서버에서 반환된 에러 정보 처리
        if (resData.detail.message === "You do not have permission") {
          alert("가족 모임 조회 실패:\n권한이 없습니다.");
        } else if (resData.detail.message === "Family not found") {
          alert("가족 모임 조회 실패:\n조회된 모임이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: resData.detail?.type,
            message: resData.detail?.message,
            input: resData.detail?.input,
          },
        };
      }
    } catch (error) {
      // 네트워크 오류 처리
      console.error("네트워크 오류 또는 기타 예외:", error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleCreateMember(event) {
    event.preventDefault();

    if (!familyIdChecked) {
      alert("가족 모임 ID를 확인해주세요.");
      return;
    }

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
      familyId: inputFamilyId.current.value,
      nickname: inputName.current.value,
    };

    userProgressStore.handleCloseModal();

    try {
      const result = await userProgressStore.handleCreateMember(payload);
      if (result.success === true) {
        // 가족 모임 등록 성공
        alert("가족 모임 등록 성공");

        inputFamilyId.current.value = "";
        inputName.current.value = "";
        setFamilyIdChecked(false);

        navigate("/accounts");
      } else {
        userProgressStore.handleOpenModal("create-member-user-info");

        console.error("가족 모임 등록 실패:", result.error);
        alert(
          `에러 발생: ${result.error.type}\n상세 메시지: ${result.error.message}`
        );
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
      alert("요청 처리 중 문제가 발생했습니다. 다시 시도해주세요.");
    }
  }

  function handleOpenFindFamily() {
    userProgressStore.handleOpenModal("find-family");
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "create-member-user-info"}
      onClose={
        userProgressStore.modalProgress === "create-member-user-info"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <div id="signup-form">
        <div className="signup-header">
          <h2>가족 모임 연결</h2>
          <button type="button" onClick={userProgressStore.handleCloseModal}>
            ⨉
          </button>
        </div>
        <div className="signup-control">
          <label htmlFor="email">가족 모임 ID</label>
          <div className="signup-wrapper">
            <input
              id="email"
              className="email-input"
              type="text"
              name="email"
              ref={inputFamilyId}
              required
            />
            {!familyIdChecked && (
              <button
                type="button"
                onClick={handleCheckFamily}
                className="email-check"
              >
                모임
                <br />
                확인
              </button>
            )}
          </div>
        </div>
        {familyIdChecked && (
          <div className="signup-control-confirm">
            <p>검색된 모임 이름: {familyIdChecked.family_name}</p>
          </div>
        )}
        <p className="signup-control">
          <label htmlFor="text">닉네임</label>
          <input type="text" ref={inputName} />
          {nameIsInvalid && (
            <div className="signup-control-error">
              <p>닉네임은 2글자 이상 32글자 이하여야 합니다.</p>
            </div>
          )}
          <button className="signup-btn" onClick={handleCreateMember}>
            가입하기
          </button>
          <button className="find-btn" onClick={handleOpenFindFamily}>
            가족 모임 ID를 잊으셨나요?
          </button>
        </p>
      </div>
    </Modal>
  );
}
