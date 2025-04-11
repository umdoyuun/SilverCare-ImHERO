import "./Accounts.css";

import { useRef, useEffect, useState, useContext } from "react";
import { useParams, useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import RouteLogin from "./RouteLogin.jsx";

export default function RegisterMemberQr() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const { familyId } = useParams();
  const [familyIdChecked, setFamilyIdChecked] = useState(false);
  const [nameIsInvalid, setNameIsInvalid] = useState(false);

  const inputName = useRef("");

  useEffect(() => {
    if (familyId !== undefined) {
      handleCheckFamily();

      if (!userProgressStore.loginUserInfo.login) {
        userProgressStore.handleOpenModal("route-login");
      } else {
        userProgressStore.handleCloseModal();
      }
    }
  }, [familyId, userProgressStore.loginUserInfo.login]);

  // 가족 구성원 조회 및 정보 저장
  async function handleCheckFamily() {
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

      console.log("가족 구성원 조회 요청:", response);

      const resData = await response.json();

      if (response.ok) {
        if (resData.message === "Family retrieved successfully") {
          console.log("가족 구성원 조회 성공");

          // 검색된 가족 모임 이름 저장
          setFamilyIdChecked(resData.result);

          return { success: true, data: resData };
        }
      } else {
        if (resData.detail.message === "Family not found") {
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
      familyId,
      nickname: inputName.current.value,
    };

    userProgressStore.handleCloseModal();

    try {
      const result = await userProgressStore.handleCreateMember(payload);
      if (result.success === true) {
        // 가족 모임 등록 성공
        alert("가족 모임 등록 성공");

        inputName.current.value = "";
        setFamilyIdChecked(false);

        navigate("/accounts");
        return;
      } else {
        userProgressStore.handleOpenModal("create-member-user-info");
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
    }
  }

  return (
    <div id="signup-form">
      <div className="signup-header">
        <h2>가족 모임 연결</h2>
        <button type="button" onClick={userProgressStore.handleCloseModal}>
          ⨉
        </button>
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
      </p>
      <RouteLogin />
    </div>
  );
}
