import "../advertisement/Advertisement.css";

import { useContext, useRef } from "react";
import { useParams, useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";
import Signup from "./Signup.jsx";

export default function LoginModal() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  const { familyId } = useParams();

  //   console.log("familyId rouete:", familyId);

  const emailInput = useRef("");
  const passwordInput = useRef("");

  async function handleLogin(event) {
    event.preventDefault();

    if (familyId === undefined) {
      alert("잘못된 접근입니다.");
      navigate("/");
    }

    const email = emailInput.current.value;
    const password = passwordInput.current.value;

    userProgressStore.handleCloseModal();

    const response = await userProgressStore.handleLogin(email, password);

    if (response.success) {
      return;
    }
    if (!response.success) {
      userProgressStore.setModalProgress("route-login");
    }
  }

  function handleShowSignUp() {
    userProgressStore.setModalProgress("sign-up");
  }

  return (
    <Modal
      id="login-modal"
      open={userProgressStore.modalProgress === "route-login"}
      onClose={
        userProgressStore.modalProgress === "route-login"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <form id="login-form" onSubmit={handleLogin}>
        <div className="login-header">
          <h2>가족 모임 등록을 하시려면 로그인이 필요합니다.</h2>
          <button
            type="button"
            onClick={() => userProgressStore.handleCloseModal()}
          >
            ⨉
          </button>
        </div>
        <div className="login-form-row">
          <div className="login-control">
            <label htmlFor="email">이메일 아이디</label>
            <input type="email" name="email" ref={emailInput} />
          </div>

          <div className="login-control">
            <label htmlFor="password">비밀번호</label>
            <input
              // id="password"
              type="password"
              name="password"
              ref={passwordInput}
            />
          </div>
        </div>

        <p className="login-form-action">
          <button type="submit" className="login-btn">
            Login
          </button>
          <button
            type="button"
            className="signup-btn"
            onClick={handleShowSignUp}
          >
            Sign Up
          </button>
        </p>
      </form>
      <Signup />
    </Modal>
  );
}
