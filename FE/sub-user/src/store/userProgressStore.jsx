import { useState, useEffect, createContext } from "react";
import { useHttp } from "../hooks/useHttp";

import { getEnvironments } from "./environmentsStore.jsx";

export const UserProgressContext = createContext({
  isActiveSideBarElem: "",
  toggleStatus: {
    notification: false,
    camera: false,
    microphone: false,
    car: false,
  },
  subUserSettings: {
    auto_login: false,
    emergency_notification: false,
  },
  sidebarIsOpened: false,
  modalProgress: "",
  selectedModalId: "",
  loginUserInfo: {
    login: false,
    userInfo: undefined,
  },
  autoLogin: false,
  familyInfo: {
    isExist: false,
    familyInfo: undefined,
    familyMember: [],
  },
  memberInfo: {
    isExist: false,
    registerData: undefined,
    selectedFamilyId: "",
  },
  DEV_API_URL: "",
  MAIN_API_URL: "",
  IMAGE_API_URL: "",
  DEV_KEY: "",
  MAIN_KEY: "",
  setIsActiveSideBarElem: () => {},
  setToggleStatus: () => {},
  setSubUserSettings: () => {},
  setSidebarIsOpened: () => {},
  setModalProgress: () => {},
  setSelectedModalId: () => {},
  setLoginUserInfo: () => {},
  setAutoLogin: () => {},
  setFamilyInfo: () => {},
  setMemberInfo: () => {},
  handleUpdateSessionLoginInfo: (userInfo) => {},
  handleActiveSideBarElem: (identifier) => {},
  handleToggleStatus: (toggle) => {},
  handleSubUserSettings: (toggle) => {},
  handleSidebarToggle: () => {},
  handleOpenModal: (identifier, id) => {},
  handleCloseModal: () => {},
  handleChangeFamilyId: (familyId) => {},
  handleLogin: (userInfo) => {},
  handleGetSession: (sessionId) => {},
  handleAutoLogin: () => {},
  handleGetUserInfo: (id) => {},
  handleLogout: () => {},
  handleCheckEmail: (email) => {},
  handleSignUp: (payload) => {},
  handleUpdateUserInfo: (payload) => {},
  handleSignOut: (password) => {},
  handleCheckFamilyExist: (userId) => {},
  handleGetFamilyName: (familyId) => {},
  handleFindFamilyInfo: (payload) => {},
  handleCreateFamily: (payload) => {},
  handleGetFamilyInfo: (familyId) => {},
  handleUpdateFamilyInfo: (newFamilyName) => {},
  handleKickMember: (memberId, password) => {},
  handleDeleteFamilyInfo: (password) => {},
  handleGetFamilyMemberInfo: (familyId) => {},
  handleCreateMember: (familyId, nickname) => {},
  handleUpdateMember: (nickname) => {},
  handleDeleteMember: (password) => {},
  handleChangePassword: (password, newPassword) => {},
});

export default function UserProgressContextProvider({ children }) {
  const { request, loading } = useHttp();

  // 사이드 바 메뉴 요소 활성화 관련
  const [isActiveSideBarElem, setIsActiveSideBarElem] = useState("home");

  // 기기 조작 관련
  const [toggleStatus, setToggleStatus] = useState({
    notification: false,
    camera: true,
    microphone: true,
    car: false,
  });

  // 보조 사용자 관리 관련
  const [subUserSettings, setSubUserSettings] = useState({
    auto_login: false,
    emergency_notification: false,
  });

  // 모바일 환경에서 사이드 바 열림 여부 관련
  const [sidebarIsOpened, setSidebarIsOpened] = useState(false);

  // 모달 열림 여부 관련
  const [modalProgress, setModalProgress] = useState("");

  // 모달 선택된 아이디 관련
  const [selectedModalId, setSelectedModalId] = useState("");

  // 로그인 정보 관리
  const [loginUserInfo, setLoginUserInfo] = useState({
    login: false,
    userInfo: undefined,
  });

  // 자동 로그인 여부 관리
  const [autoLogin, setAutoLogin] = useState(false);

  // 가족 정보 관리
  const [familyInfo, setFamilyInfo] = useState({
    isExist: false,
    familyInfo: undefined,
    familyMember: undefined,
  });

  const [memberInfo, setMemberInfo] = useState({
    isExist: false,
    registerData: undefined,
    selectedFamilyId: undefined,
  });

  // ======================================================================
  // env 관련
  let DEV_API_URL = import.meta.env.VITE_DEV_API;
  let MAIN_API_URL = import.meta.env.VITE_MAIN_API;
  let IMAGE_API_URL = import.meta.env.VITE_IMAGE_API;
  let DEV_KEY = import.meta.env.VITE_DEV_KEY;
  let MAIN_KEY = import.meta.env.VITE_MAIN_KEY;

  if (DEV_API_URL === undefined) DEV_API_URL = getEnvironments("DEV_API_URL");
  if (MAIN_API_URL === undefined)
    MAIN_API_URL = getEnvironments("MAIN_API_URL");
  if (IMAGE_API_URL === undefined)
    IMAGE_API_URL = getEnvironments("IMAGE_API_URL");
  if (DEV_KEY === undefined) DEV_KEY = getEnvironments("DEV_KEY");
  if (MAIN_KEY === undefined) MAIN_KEY = getEnvironments("MAIN_KEY");
  // ======================================================================

  function handleUpdateSessionLoginInfo(userInfo) {
    setLoginUserInfo({
      login: userInfo.login,
      userInfo: userInfo.userInfo,
    });

    // 로그인 정보 저장
    sessionStorage.setItem("loginUserInfo", JSON.stringify(userInfo));
  }

  // 사이드 바 요소 활성화
  function handleActiveSideBarElem(identifier) {
    setIsActiveSideBarElem(identifier);
    sessionStorage.setItem("isActiveSideBarElem", identifier); // 활성화된 요소 저장

    setSidebarIsOpened(false); // 모바일 환경일 경우 사이드 바 닫음
  }

  // 기기 조작 관련 함수
  function handleToggleStatus(toggle) {
    setToggleStatus((prevStatus) => {
      return { ...prevStatus, [toggle]: !prevStatus[toggle] };
    });
  }

  function handleSubUserSettings(toggle) {
    setSubUserSettings((prevSettings) => {
      return { ...prevSettings, [toggle]: !prevSettings[toggle] };
    });
  }

  // 모바일 환경에서 사이드 바 열고 닫기
  function handleSidebarToggle() {
    setSidebarIsOpened((prev) => !prev);
  }

  // 모달 열기
  function handleOpenModal(identifier, id) {
    // console.log("모달 열기:", identifier, id);
    setModalProgress(identifier);
    setSelectedModalId(id);
  }

  // 모달 초기화
  function handleCloseModal() {
    setModalProgress("");
    setSelectedModalId("");
  }

  // 활성화 가족 모임 변경
  function handleChangeFamilyId(familyId) {
    setMemberInfo((prev) => {
      return { ...prev, selectedFamilyId: familyId };
    });
  }

  // 로그인
  async function handleLogin(email, password) {
    try {
      const response = await request(`${DEV_API_URL}/auth/login`, "POST", {
        email,
        password,
      });

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Login successful") {
          // console.log("로그인 성공", resData);

          // 자동 로그인 설정
          const autoLoginConfirm =
            window.confirm("자동 로그인을 설정하시겠습니까?");
          if (autoLoginConfirm) {
            await handleAutoLogin();
            const autoLoginUserInfo = {
              login: true,
              userInfo: resData.result.user_data,
            };
            localStorage.setItem("session_id", resData.result.session_id);
            localStorage.setItem(
              "loginUserInfo",
              JSON.stringify(autoLoginUserInfo)
            );
          } else {
            sessionStorage.setItem("session_id", resData.result.session_id);
          }

          // 로그인 정보 저장
          await handleUpdateSessionLoginInfo({
            login: true,
            userInfo: resData.result.user_data,
          });

          return { success: true, data: resData };
        }
      } else {
        console.error("로그인 실패:", response.error);
        alert("로그인 실패\n아이디 또는 비밀번호를 확인해주세요.");
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  async function handleGetSession(sessionId) {
    if (!sessionId) return;

    try {
      const response = await request(`${DEV_API_URL}/auth/check`, "POST", {
        session_id: sessionId,
      });

      const resData = response.data;
      if (response.success) {
        if (resData.message === "Permission check successful") {
          // console.log("세션 확인 성공", resData);
          setLoginUserInfo({
            login: true,
            userInfo: resData.result.user_data,
          });
          return { success: true, data: resData };
        }
      } else {
        console.error("세션 확인 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  async function handleAutoLogin() {
    try {
      const response = await request(`${DEV_API_URL}/auth/auto-login`, "PATCH");

      const resData = response.data;

      if (response.success) {
        if (resData.message === "Auto login set successfully") {
          // console.log("자동 로그인 설정 성공", resData);

          setAutoLogin(true);

          return { success: true, data: resData };
        }
      } else {
        alert("자동 로그인 설정 실패\n권한이 없습니다.");
      }
    } catch (error) {
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

  // 최신 회원 정보 조회
  async function handleGetUserInfo(id) {
    try {
      const response = await request(`${DEV_API_URL}/accounts/${id}`, "GET");

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Account retrieved successfully") {
          // console.log("회원 정보 조회 성공", resData);

          await handleUpdateSessionLoginInfo({
            login: true,
            userInfo: resData.result,
          });

          return { success: true, data: resData };
        }
      } else {
        console.error("회원 정보 조회 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error?.type || "unknown_error",
            message: response.error?.message || "알 수 없는 오류 발생",
            input: response.error?.input,
          },
        };
      }
    } catch (error) {
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

  // 로그아웃
  async function handleLogout() {
    try {
      const response = await request(`${DEV_API_URL}/auth/logout`, "POST");

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Logout successful") {
          // console.log("로그아웃 성공");

          setLoginUserInfo({
            login: false,
            userInfo: undefined,
          });

          // 세션 스토리지에서 로그인 정보 삭제
          sessionStorage.removeItem("loginUserInfo");
          sessionStorage.removeItem("session_id");

          // 로컬 스토리지에서 로그인 정보 삭제
          localStorage.removeItem("loginUserInfo");
          localStorage.removeItem("session_id");

          // 사이드바 관리
          setIsActiveSideBarElem("accounts");

          // 기본 경로로 이동
          window.location.href = "/";

          // 모달 초기화
          await handleCloseModal();

          // 사이드바 닫기 (동기 처리)
          setSidebarIsOpened(false);

          // 기기 활성화 요소 초기화 (동기 처리)
          sessionStorage.removeItem("isActiveSideBarElem");

          return { success: true, data: resData };
        }
      } else {
        console.error("로그아웃 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error?.type || "unknown_error",
            message: response.error?.message || "알 수 없는 오류 발생",
            input: response.error?.input,
          },
        };
      }
    } catch (error) {
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

  // 이메일 중복 확인
  async function handleCheckEmail(email) {
    try {
      const response = await request(
        `${DEV_API_URL}/accounts/check-email`,
        "POST",
        { email: email }
      );

      // console.log(response);

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Email is available") {
          // console.log("이메일 사용 가능:", email);
          return true;
        }
      } else {
        if (response.error.message === "Email is already in use") {
          console.error("이메일 이미 존재:", response.error.input.email);
          return false;
        }
      }
    } catch (error) {
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

  // 회원 가입
  async function handleSignUp(payload) {
    try {
      const response = await request(
        `${DEV_API_URL}/accounts`,
        "POST",
        payload
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "New account created successfully") {
          // console.log("회원 가입 성공", resData.result.id);

          handleLogin(payload.email, payload.password); // 회원 가입 후 자동 로그인

          return { success: true, data: resData };
        }
      } else {
        console.error("회원 가입 실패:", response.error);
        if (response.error?.message === "Email is already in use") {
          alert("회원 가입 실패:\n이미 사용 중인 이메일입니다.");
        } else if (response.error?.type === "invalid value") {
          alert("회원 가입 실패:\n입력값이 올바르지 않습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error?.type || "unknown_error",
            message: response.error?.message || "알 수 없는 오류 발생",
            input: response.error?.input,
          },
        };
      }
    } catch (error) {
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

  // 회원 정보 수정
  async function handleUpdateUserInfo(payload) {
    const isConfirmed = window.confirm(
      '정말 회원 정보를 수정하시겠습니까?\n"확인"을 누르면 수정이 완료됩니다.'
    );
    if (!isConfirmed) {
      // console.log("회원 수정 취소");
      return {
        success: false,
        error: { type: "canceled", message: "사용자가 취소했습니다." },
      };
    }

    try {
      const response = await request(
        `${DEV_API_URL}/accounts/${loginUserInfo.userInfo.id}`,
        "PATCH",
        payload
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Account updated successfully") {
          // console.log("회원 정보 수정 성공");

          // 최신 회원 정보 가져오기
          const updatedUser = await handleGetUserInfo(
            loginUserInfo.userInfo.id
          );
          // console.log("최신 회원 정보 응답:", updatedUser);

          if (!updatedUser || !updatedUser.success) {
            console.error("회원 정보 갱신 실패:", updatedUser);
            return {
              success: false,
              error: { message: "회원 정보 조회 실패" },
            };
          }

          // 상태 업데이트
          const newUserInfo = {
            login: true,
            userInfo: updatedUser.data.result,
          };

          setLoginUserInfo(newUserInfo);
          sessionStorage.setItem("loginUserInfo", JSON.stringify(newUserInfo));

          return { success: true, data: resData };
        }
      } else {
        // console.log("회원 정보 수정 실패:", response.error);
        if (response.error.type === "invalid value") {
          alert("회원 정보 수정 실패:\n입력값이 올바르지 않습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 회원 탈퇴
  async function handleSignOut(password) {
    const isConfirmed = window.confirm(
      `정말 회원 탈퇴하시겠습니까?\n탈퇴 시 복구가 불가능합니다.`
    );
    if (!isConfirmed) {
      // console.log("회원 탈퇴 취소");
      return {
        success: false,
        error: { type: "canceled", message: "사용자가 취소했습니다." },
      };
    }

    try {
      const response = await request(
        `${DEV_API_URL}/accounts/${loginUserInfo.userInfo.id}`,
        "DELETE",
        { password: password }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Account deleted successfully") {
          // console.log("회원 탈퇴 성공");

          setLoginUserInfo({
            login: false,
            userInfo: undefined,
          });

          // 세션 스토리지에서 로그인 정보 삭제
          sessionStorage.removeItem("loginUserInfo");
          sessionStorage.removeItem("session_id");

          // 로컬 스토리지에서 로그인 정보 삭제
          localStorage.removeItem("loginUserInfo");
          localStorage.removeItem("session_id");

          // 사이드바 관리
          setIsActiveSideBarElem("accounts");

          // 기본 경로로 이동
          window.location.href = "/";

          // 모달 초기화
          await handleCloseModal();

          // 사이드바 닫기
          setSidebarIsOpened(false);

          // 기기 활성화 요소 초기화
          sessionStorage.removeItem("isActiveSideBarElem");

          return { success: true, data: resData };
        }
      } else {
        // console.log("회원 탈퇴 실패:", response.error);

        if (response.error.type === "unauthorized") {
          alert("회원 탈퇴 실패:\n잘못된 비밀번호 입력");
        } else if (response.error.type === "Password is required") {
          alert("회원 탈퇴 실패:\n비밀번호 입력은 필수입니다");
        }

        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 모임 존재 여부 확인
  async function handleCheckFamilyExist(userId) {
    try {
      const response = await request(
        `${DEV_API_URL}/families/check-exist`,
        "POST",
        { id: userId }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Family exists") {
          // console.log("가족 존재함");

          // 가족 정보 조회
          await handleGetFamilyInfo(resData.result.family_id);

          return { success: true, data: resData };
        }
      } else {
        // console.log("가족 모임 존재하지 않음");
        return { success: false, data: response.error };
      }
    } catch (error) {
      console.error("네트워크 오류 또는 기타 예외:", error);
      throw new Error("요청을 처리하는 동안 오류가 발생했습니다.");
    }
  }

  // 가족 생성
  async function handleCreateFamily(payload) {
    const isConfirmed = window.confirm(
      `${payload.family_name}으로 가족 모임 생성을 하시겠습니까?\n확인을 누르면 가족이 생성됩니다.`
    );
    if (!isConfirmed) {
      // console.log("가족 모임 생성 취소");
      return {
        success: false,
        error: { type: "canceled", message: "사용자가 취소했습니다." },
      };
    }

    try {
      const response = await request(
        `${DEV_API_URL}/families`,
        "POST",
        payload
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "New family created successfully") {
          // console.log("가족 모임 생성 성공");

          // 가족 생성 후 가족 정보 조회
          await handleGetFamilyInfo(resData.result.id);

          return { success: true, data: resData };
        }
      } else {
        console.error("가족 생성 실패:", response.error);
        if (response.error.type === "already exists") {
          alert("가족 생성 실패:\n이미 존재하는 가족 이름입니다.");
        } else if (response.error.type === "invalid value") {
          alert("가족 생성 실패:\n입력값이 올바르지 않습니다.");
        } else if (response.error.type === "can not access") {
          alert("가족 생성 실패:\n권한이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
      console.error("네트워크 오류 또는 기타 예외:", error);
      throw new Error("요청을 처리하는 동안 오류가 발생했습니다.");
    }
  }

  // 가족 모임 정보보 조회 및 정보 저장
  async function handleGetFamilyInfo(familyId) {
    try {
      const response = await request(
        `${DEV_API_URL}/families/${familyId}`,
        "GET"
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Family retrieved successfully") {
          // console.log("가족 모임 정보 조회 성공");

          // 정보 갱신
          setFamilyInfo({
            isExist: true,
            familyInfo: resData.result,
          });

          // 가족 구성원 정보 조회
          await handleGetFamilyMemberInfo(resData.result.id);

          return { success: true, data: resData };
        }
      } else {
        console.error("가족 모임 정보 조회 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
      console.error("네트워크 오류 또는 기타 예외:", error);
      return {
        type: "network_error",
        message: "네트워크 오류가 발생했습니다.",
      };
    }
  }

  // 가족 정보 수정
  async function handleUpdateFamilyInfo(newFamilyName) {
    const isConfirmed = window.confirm(
      "정말 가족 모임 정보를 변경하시겠습니까?\n확인을 누르면 변경됩니다."
    );
    if (!isConfirmed) {
      // console.log("가족 정보 변경 취소");
      return {
        success: false,
        error: { type: "canceled", message: "사용자가 취소했습니다." },
      };
    }

    try {
      const response = await request(
        `${DEV_API_URL}/families/${familyInfo.familyInfo.id}`,
        "PATCH",
        { family_name: newFamilyName }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Family updated successfully") {
          // console.log("가족 모임 정보 수정 성공");

          // 가족 정보 다시 조회
          await handleGetFamilyInfo(familyInfo.familyInfo.id);

          return { success: true, data: resData };
        }
      } else {
        console.error("가족 모임 정보 수정 실패:", response.error);
        if (response.error.type === "not found") {
          alert("가족 정보 수정 실패:\n가족을 찾을 수 없습니다.");
        } else if (response.error.type === "can not access") {
          alert("가족 정보 수정 실패:\n권한이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  async function handleKickMember(memberId, password) {
    try {
      const response = await request(
        `${DEV_API_URL}/members/kick/${memberId}`,
        "DELETE",
        { password: password }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Member kicked successfully") {
          // console.log("구성원 추방 성공");

          // 가족 구성원 정보 다시 조회
          await handleGetFamilyMemberInfo(familyInfo.familyInfo.id);

          return { success: true, data: resData };
        }
      } else {
        console.error("구성원 추방 실패:", response.error);
        if (response.error.message === "Invalid password") {
          alert("구성원 추방 실패:\n잘못된 비밀번호 입력");
        } else if (response.error.message === "You do not have permission") {
          alert("구성원 추방 실패:\n권한이 없습니다.");
        } else if (response.error.message === "Member not found") {
          alert("구성원 추방 실패:\n구성원을 찾을 수 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 정보 삭제
  async function handleDeleteFamilyInfo(password) {
    const isConfirmed = window.confirm(
      "정말 가족 모임을 삭제하시겠습니까?\n삭제 시 복구가 불가능합니다."
    );
    if (!isConfirmed) {
      // console.log("가족 정보 삭제 취소");
      return {
        success: false,
        error: { type: "canceled", message: "삭제 취소." },
      };
    }

    try {
      const response = await request(
        `${DEV_API_URL}/families/${familyInfo.familyInfo.id}`,
        "DELETE",
        { password: password }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Family deleted successfully") {
          // console.log("가족 모임 삭제 성공");

          // 가족 정보 상태 초기화
          setFamilyInfo({
            isExist: false,
            familyInfo: undefined,
          });

          return { success: true, data: resData };
        }
      } else {
        console.error("가족 모임 삭제 실패:", response.error);
        if (response.error.message === "Invalid password") {
          alert("가족 모임 삭제 실패:\n잘못된 비밀번호 입력");
        } else if (response.error.message === "Family not found") {
          alert("가족 모임 삭제 실패:\n가족을 찾을 수 없습니다.");
        } else if (response.error.message === "You do not have permission") {
          alert("가족 모임 삭제 실패:\n권한이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 등록된 가족 모임 목록 조회
  async function handleCheckFamilyList() {
    try {
      const response = await request(
        `${DEV_API_URL}/members?userId=${encodeURIComponent(
          loginUserInfo.userInfo.id
        )}`,
        "GET"
      );

      const resData = response.data;

      if (response.success) {
        if (resData.message === "All members retrieved successfully") {
          // console.log("가족 목록 조회 성공");

          // 가족 이름 검색 및 저장
          const familyIds = resData.result;
          const newRegisterData = await handleGetFamilyName(familyIds);

          if (newRegisterData) {
            // 정보 갱신
            setMemberInfo({
              isExist: true,
              registerData: newRegisterData,
              selectedFamilyId: newRegisterData[0].family_id,
            });
          } else {
            setMemberInfo({
              isExist: true,
              registerData: resData.result,
              selectedFamilyId: resData.result[0].family_id,
            });
          }

          return { success: true, data: resData };
        } else if (resData.message === "No members found") {
          // console.log("조회된 가족 목록 없음");

          // 정보 갱신
          setMemberInfo({
            isExist: false,
            registerData: undefined,
            selectedFamilyId: undefined,
          });
        }
        return { success: true, data: resData };
      } else {
        console.error("가족 목록 조회 실패:", resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 이름 검색 및 저장
  async function handleGetFamilyName(familyIds) {
    if (!familyIds) return;

    try {
      const newRegisterData = [];

      for (let i = 0; i < familyIds.length; i++) {
        const response = await request(
          `${DEV_API_URL}/families/${familyIds[i].family_id}`,
          "GET"
        );

        if (response.success) {
          const resData = response.data;

          if (resData.message === "Family retrieved successfully") {
            newRegisterData.push({
              ...familyIds[i],
              family_name: resData.result.family_name,
              main_user_id: resData.result.main_user,
            });
          }
        } else {
          console.error("가족 이름 조회 실패:", response.error);
          return familyIds;
        }
      }
      // 반환
      return newRegisterData;
    } catch (error) {
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

  // 가족 모임 찾기
  async function handleFindFamilyInfo(payload) {
    try {
      const response = await request(
        `${DEV_API_URL}/families/find`,
        "POST",
        payload
      );

      if (response.success) {
        const resData = response.data;

        if (
          resData.message === "Families found successfully" &&
          resData.result.length > 0
        ) {
          // console.log("가족 모임 찾기 성공");
          return { success: true, data: resData.result[0] };
        }
      } else {
        console.error("가족 모임 찾기 실패:", response.error);
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 구성원 조회
  async function handleGetFamilyMemberInfo(familyId) {
    try {
      const response = await request(
        `${DEV_API_URL}/members?familyId=${encodeURIComponent(familyId)}`,
        "GET"
      );

      const resData = response.data;

      if (response.success) {
        if (resData.message === "All members retrieved successfully") {
          // console.log("가족 모임 구성원 조회 성공");
          console.log(resData.result);

          // 정보 갱신
          setFamilyInfo((prev) => {
            return {
              ...prev,
              familyMember: resData.result,
            };
          });
        } else if (resData.message === "No members found") {
          // console.log("가족 모임 구성원 없음");

          // 정보 갱신
          setFamilyInfo((prev) => {
            return {
              ...prev,
              familyMember: [],
            };
          });
        }
        return { success: true, data: resData };
      } else {
        console.error("가족 구성원 조회 실패:", resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 구성원 추가
  async function handleCreateMember(payload) {
    const { familyId, nickname } = payload;

    try {
      const response = await request(`${DEV_API_URL}/members`, "POST", {
        family_id: familyId,
        user_id: loginUserInfo.userInfo.id,
        nickname: nickname,
      });

      if (response.success) {
        const resData = response.data;

        if (resData.message === "New member created successfully") {
          // console.log("가족 구성원 추가 성공");

          // 정보 갱신
          await handleCheckFamilyList();

          return { success: true, data: resData };
        }
      } else {
        console.error("가족 구성원 추가 실패:", response.error);
        if (response.error.message === "Member already exists in family") {
          alert("가족 구성원 추가 실패:\n이미 가입된 구성원입니다.");
        } else if (response.error.message === "Family not found") {
          alert("가족 구성원 추가 실패:\n가족을 찾을 수 없습니다.");
        } else if (response.error.message === "You do not have permission") {
          alert("가족 구성원 추가 실패:\n권한이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 구성원 닉네임 수정
  async function handleUpdateMember(nickname) {
    try {
      const response = await request(
        `${DEV_API_URL}/members/${selectedModalId}`,
        "PATCH",
        { nickname: nickname }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Member updated successfully") {
          // console.log("구성원 정보 수정 성공");

          // 정보 갱신
          await handleCheckFamilyList();

          return { success: true, data: resData };
        }
      } else {
        console.error("구성원 정보 수정 실패:", response.error);
        if (response.error.message === "Member not found") {
          alert("구성원 정보 수정 실패:\n구성원을 찾을 수 없습니다.");
        } else if (response.error.message === "You do not have permission") {
          alert("구성원 정보 수정 실패:\n권한이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  // 가족 구성원 삭제
  async function handleDeleteMember(password) {
    try {
      const response = await request(
        `${DEV_API_URL}/members/${selectedModalId}`,
        "DELETE",
        { password: password }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Member deleted successfully") {
          // console.log("구성원 삭제 성공");

          // 정보 갱신
          await handleCheckFamilyList();

          return { success: true, data: resData };
        }
      } else {
        console.error("구성원 삭제 실패:", response.error);
        if (response.error.message === "Invalid password") {
          alert("구성원 삭제 실패:\n잘못된 비밀번호 입력");
        } else if (response.error.message === "Member not found") {
          alert("구성원 삭제 실패:\n구성원을 찾을 수 없습니다.");
        } else if (response.error.message === "You do not have permission") {
          alert("구성원 삭제 실패:\n권한이 없습니다.");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  async function handleChangePassword(password, newPassword) {
    const isConfirmed = window.confirm(
      "정말 비밀번호를 변경하시겠습니까?\n변경 시 로그아웃됩니다."
    );
    if (!isConfirmed) {
      // console.log("비밀번호 변경 취소");
      return {
        success: false,
        error: { type: "canceled", message: "사용자가 취소했습니다." },
      };
    }

    try {
      const response = await request(
        `${DEV_API_URL}/auth/change-password`,
        "PATCH",
        {
          user_id: loginUserInfo.userInfo.id,
          current_password: password,
          new_password: newPassword,
        }
      );

      if (response.success) {
        const resData = response.data;

        if (resData.message === "Password changed successfully") {
          // console.log("비밀번호 변경 성공");

          // 로그아웃
          await handleLogout();

          return { success: true, data: resData };
        }
      } else {
        console.error("비밀번호 변경 실패:", response.error);
        if (response.error.message === "You do not have permission") {
          alert("비밀번호 변경 실패:\n권한이 없습니다.");
        } else if (response.error.message === "User not found") {
          alert("비밀번호 변경 실패:\n사용자를 찾을 수 없습니다.");
        } else if (response.error.message === "Invalid password") {
          alert("비밀번호 변경 실패:\n잘못된 비밀번호 입력");
        }
        return {
          success: false,
          error: {
            type: response.error.type,
            message: response.error.message,
          },
        };
      }
    } catch (error) {
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

  const ctxValue = {
    loading,
    isActiveSideBarElem,
    toggleStatus,
    subUserSettings,
    sidebarIsOpened,
    modalProgress,
    selectedModalId,
    loginUserInfo,
    autoLogin,
    familyInfo,
    memberInfo,
    DEV_API_URL,
    MAIN_API_URL,
    IMAGE_API_URL,
    DEV_KEY,
    MAIN_KEY,
    setIsActiveSideBarElem,
    setToggleStatus,
    setSubUserSettings,
    setSidebarIsOpened,
    setModalProgress,
    setSelectedModalId,
    setLoginUserInfo,
    setAutoLogin,
    setFamilyInfo,
    setMemberInfo,
    handleUpdateSessionLoginInfo,
    handleActiveSideBarElem,
    handleToggleStatus,
    handleSubUserSettings,
    handleSidebarToggle,
    handleOpenModal,
    handleCloseModal,
    handleChangeFamilyId,
    handleLogin,
    handleAutoLogin,
    handleGetSession,
    handleGetUserInfo,
    handleLogout,
    handleCheckEmail,
    handleSignUp,
    handleSignOut,
    handleUpdateUserInfo,
    handleCheckFamilyExist,
    handleCreateFamily,
    handleGetFamilyInfo,
    handleUpdateFamilyInfo,
    handleKickMember,
    handleDeleteFamilyInfo,
    handleCheckFamilyList,
    handleGetFamilyName,
    handleFindFamilyInfo,
    handleGetFamilyMemberInfo,
    handleCreateMember,
    handleUpdateMember,
    handleDeleteMember,
    handleChangePassword,
  };

  return (
    <UserProgressContext.Provider value={ctxValue}>
      {children}
    </UserProgressContext.Provider>
  );
}
