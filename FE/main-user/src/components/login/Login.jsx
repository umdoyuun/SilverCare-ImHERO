import { useRef, useState, useContext, useEffect } from "react";
import { UserProgressContext } from "../../store/userProgressStore.jsx";
import { StoreContext } from "../../store/store.jsx";
import "./Login.css";

export default function Login() {
  const userProgressStore = useContext(UserProgressContext);
  const mainStore = useContext(StoreContext)

  // 유효성 검사 상태
  const [formIsInvalid, setFormIsInvalid] = useState({
    email: false,
    password: false,
  });

  const emailInput = useRef("");
  const passwordInput = useRef("");

  /////

  const [showWelcome, setShowWelcome] = useState(true);
  const [showForm, setShowForm] = useState(false);

  useEffect(() => {
    // 1.5초 후 환영 문구를 위로 이동시키고, 로그인 폼을 나타나게 함
    setTimeout(() => {
      setShowWelcome(false);
      setShowForm(true);
    }, 1500);
  }, []);

  /////

  async function handleLogin(event) {
    event.preventDefault();

    // 이메일 유효성 검사
    const emailIsInvalid = !emailInput.current.value.includes("@");
    if (emailIsInvalid) {
      setFormIsInvalid((prevForm) => {
        alert("유효한 이메일을 입력해 주세요.");
        return { ...prevForm, email: true };
      });
      return;
    } else {
      setFormIsInvalid((prevForm) => {
        return { ...prevForm, email: false };
      });
    }

    // 비밀번호 유효성 검사
    const passwordIsInvalid = passwordInput.current.value.length < 8;
    if (passwordIsInvalid) {
      setFormIsInvalid((prevForm) => {
        alert("비밀번호는 8자 이상입니다.");
        return { ...prevForm, password: true };
      });
      return;
    } else {
      setFormIsInvalid((prevForm) => {
        return { ...prevForm, password: false };
      });
    }

    const email = emailInput.current.value;
    const password = passwordInput.current.value;

    mainStore.handleModalClose();

    const response = await userProgressStore.handleLogin(email, password);

    // 로그인 실패 시
    if (!response.success) {
      if (response.error.message === "Invalid email or password") {
        alert("이메일 또는 비밀번호가 일치하지 않습니다.");
      } else {
        alert(`로그인 실패:\n${response.error.message}`);
      }
    }
  }

  return (
    <>
      <div className={`welcome-text ${showWelcome ? "centered" : "moved-up"}`}>
        <h2>영웅이네 오신 것을 환영합니다.</h2>
      </div>
      <form id="login-form" className={showForm ? "fade-in" : ""} onSubmit={handleLogin}>
        <div className="login-form-row">
          <div className="login-control">
            <label htmlFor="email">이메일 아이디</label>
            <input type="email" name="email" ref={emailInput} />
          </div>

          <div className="login-control">
            <label htmlFor="password">비밀번호</label>
            <input type="password" name="password" ref={passwordInput} />
          </div>
        </div>

        <button type="submit" className="login-btn">
          로그인
        </button>
      </form>
    </>
  );
}