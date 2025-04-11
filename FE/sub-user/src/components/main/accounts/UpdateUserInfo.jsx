import "./Accounts.css";

import { useRef, useState, useContext } from "react";
import { useNavigate } from "react-router-dom";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import Modal from "../../modal/Modal.jsx";

const regions = [
  {
    name: "서울특별시",
    cities: [
      "강남구",
      "강동구",
      "강북구",
      "강서구",
      "관악구",
      "광진구",
      "구로구",
      "금천구",
      "노원구",
      "도봉구",
      "동대문구",
      "동작구",
      "마포구",
      "서대문구",
      "서초구",
      "성동구",
      "성북구",
      "송파구",
      "양천구",
      "영등포구",
      "용산구",
      "은평구",
      "종로구",
      "중구",
      "중랑구",
    ],
  },
  {
    name: "경기도",
    cities: [
      "수원시",
      "성남시",
      "고양시",
      "용인시",
      "부천시",
      "안산시",
      "안양시",
      "남양주시",
      "화성시",
      "평택시",
      "의정부시",
      "시흥시",
      "파주시",
      "김포시",
      "광주시",
      "군포시",
      "광명시",
      "이천시",
      "양주시",
      "오산시",
      "구리시",
      "안성시",
      "포천시",
      "의왕시",
      "하남시",
      "여주시",
    ],
  },
  {
    name: "인천광역시",
    cities: [
      "계양구",
      "미추홀구",
      "남동구",
      "동구",
      "부평구",
      "서구",
      "연수구",
      "중구",
      "강화군",
      "옹진군",
    ],
  },
  {
    name: "경상북도",
    cities: [
      "포항시",
      "경주시",
      "김천시",
      "안동시",
      "구미시",
      "영주시",
      "영천시",
      "상주시",
      "문경시",
      "경산시",
      "군위군",
      "의성군",
      "청송군",
      "영양군",
      "영덕군",
      "청도군",
      "고령군",
      "성주군",
      "칠곡군",
      "예천군",
      "봉화군",
      "울진군",
      "울릉군",
    ],
  },
  {
    name: "경상남도",
    cities: [
      "창원시",
      "진주시",
      "통영시",
      "사천시",
      "김해시",
      "밀양시",
      "거제시",
      "양산시",
      "의령군",
      "함안군",
      "창녕군",
      "고성군",
      "남해군",
      "하동군",
      "산청군",
      "함양군",
      "거창군",
      "합천군",
    ],
  },
  {
    name: "부산광역시",
    cities: [
      "강서구",
      "금정구",
      "기장군",
      "남구",
      "동구",
      "동래구",
      "부산진구",
      "북구",
      "사상구",
      "사하구",
      "서구",
      "수영구",
      "연제구",
      "영도구",
      "중구",
      "해운대구",
    ],
  },
  {
    name: "대구광역시",
    cities: [
      "남구",
      "달서구",
      "달성군",
      "동구",
      "북구",
      "서구",
      "수성구",
      "중구",
    ],
  },
  {
    name: "광주광역시",
    cities: ["광산구", "남구", "동구", "북구", "서구"],
  },
  {
    name: "대전광역시",
    cities: ["대덕구", "동구", "서구", "유성구", "중구"],
  },
  {
    name: "울산광역시",
    cities: ["남구", "동구", "북구", "울주군", "중구"],
  },
  {
    name: "강원도",
    cities: [
      "춘천시",
      "원주시",
      "강릉시",
      "동해시",
      "태백시",
      "속초시",
      "삼척시",
      "홍천군",
      "횡성군",
      "영월군",
      "평창군",
      "정선군",
      "철원군",
      "화천군",
      "양구군",
      "인제군",
      "고성군",
      "양양군",
    ],
  },
  {
    name: "충청북도",
    cities: [
      "청주시",
      "충주시",
      "제천시",
      "보은군",
      "옥천군",
      "영동군",
      "증평군",
      "진천군",
      "괴산군",
      "음성군",
      "단양군",
    ],
  },
  {
    name: "충청남도",
    cities: [
      "천안시",
      "공주시",
      "보령시",
      "아산시",
      "서산시",
      "논산시",
      "계룡시",
      "당진시",
      "금산군",
      "부여군",
      "서천군",
      "청양군",
      "홍성군",
      "예산군",
      "태안군",
    ],
  },
  {
    name: "전라북도",
    cities: [
      "전주시",
      "군산시",
      "익산시",
      "정읍시",
      "남원시",
      "김제시",
      "완주군",
      "진안군",
      "무주군",
      "장수군",
      "임실군",
      "순창군",
      "고창군",
      "부안군",
    ],
  },
  {
    name: "전라남도",
    cities: [
      "목포시",
      "여수시",
      "순천시",
      "나주시",
      "광양시",
      "담양군",
      "곡성군",
      "구례군",
      "고흥군",
      "보성군",
      "화순군",
      "장흥군",
      "강진군",
      "해남군",
      "영암군",
      "무안군",
      "함평군",
      "영광군",
      "장성군",
      "완도군",
      "진도군",
      "신안군",
    ],
  },
  {
    name: "제주특별자치도",
    cities: ["제주시", "서귀포시"],
  },
];

export default function UpdateUserInfo() {
  const userProgressStore = useContext(UserProgressContext);
  const navigate = useNavigate();

  // 유효성 검사 상태
  const [formIsInvalid, setFormIsInvalid] = useState({
    email: false,
    emailCheck: "",
  });

  const userInformation = userProgressStore.loginUserInfo.userInfo;

  const emailInput = useRef("");

  const addressParts = userInformation.address
    ? userInformation.address.toString().split(" ")
    : ["", ""];

  const initialState = addressParts[0] || ""; // 시/도 초기값
  const initialCity = addressParts[1] || ""; // 시/군/구 초기값

  const [selectedState, setSelectedState] = useState(initialState);
  const [cities, setCities] = useState(
    regions.find((region) => region.name === initialState)?.cities || []
  );
  const [selectedCity, setSelectedCity] = useState(initialCity);

  // 이메일 확인 함수
  async function handleEmailCheck() {
    const enteredEmail = emailInput.current.value;

    // 이메일이 입력되지 않았을 때 중단
    if (enteredEmail.length === 0) {
      return;
    }

    // 이메일 형식 유효성 검사
    const emailIsInvalid = !enteredEmail.includes("@");

    if (emailIsInvalid) {
      return; // 이메일 형식이 잘못되면 중단
    }

    // 이메일 상태 업데이트
    setFormIsInvalid((prevForm) => ({
      ...prevForm,
      email: emailIsInvalid,
      emailCheck: emailIsInvalid ? null : prevForm.emailCheck,
    }));

    try {
      console.log("확인 호출");
      const isEmailAvailable = await userProgressStore.handleCheckEmail(
        enteredEmail
      );

      if (isEmailAvailable === true) {
        setFormIsInvalid((prevForm) => ({
          ...prevForm,
          emailCheck: "verified", // 이메일 사용 가능
        }));
      } else if (isEmailAvailable === false) {
        setFormIsInvalid((prevForm) => ({
          ...prevForm,
          emailCheck: "not-available", // 이메일 사용 불가능
        }));
      } else {
        console.error("Email check result is null. Unable to verify.");
        setFormIsInvalid((prevForm) => ({
          ...prevForm,
          emailCheck: "not-verified", // 이메일 확인 결과가 없을 때
        }));
      }
    } catch (error) {
      console.error("Email check error:", error?.message || error);
      setFormIsInvalid((prevForm) => ({
        ...prevForm,
        emailCheck: "not-verified", // 이메일 확인 실패
      }));
    }
  }

  function handleStateChange(event) {
    const stateName = event.target.value;
    setSelectedState(stateName);

    const region = regions.find((region) => region.name === stateName);
    setCities(region ? region.cities : []);
    setSelectedCity(""); // 시/군/구 초기화
  }

  function handleCityChange(event) {
    setSelectedCity(event.target.value);
  }

  async function handleSubmit(event) {
    event.preventDefault();

    const fd = new FormData(event.target);
    const data = Object.fromEntries(fd.entries());

    let isValid = true;
    let newFormState = { ...formIsInvalid };

    // 이메일 수정을 할 경우 유효성 검사
    if (data.email.length > 0) {
      if (!data.email.includes("@")) {
        newFormState.email = true;
        isValid = false;
      } else {
        newFormState.email = false;
      }

      // 이메일 중복 확인 여부 검사
      if (formIsInvalid.emailCheck !== "verified") {
        newFormState.emailCheck = "not-verified";
        isValid = false;
      } else {
        newFormState.emailCheck = "verified";
      }

      if (!isValid) {
        setFormIsInvalid(newFormState);
        return;
      }
    }

    if (data.user_name.length < 2 || data.user_name.length > 32) {
      alert("이름은 2자 이상 32자 이하로 입력해주세요.");
      return;
    }

    // 입력받은 데이터 객체화
    const payload = {
      ...(data.email && { email: data.email }), // email이 있으면 추가
      role: data.role,
      user_name: data["user_name"],
      birth_date: {
        year: Number(data["birth_date"].slice(0, 4)),
        month: Number(data["birth_date"].slice(5, 7)),
        day: Number(data["birth_date"].slice(8, 10)),
      },
      gender: data.gender,
      address: data.state + " " + data.city,
    };

    userProgressStore.handleCloseModal();

    // 백 요청 전송
    try {
      const result = await userProgressStore.handleUpdateUserInfo(payload);

      if (result.success) {
        console.log("회원 정보 수정 성공:", result.data);
        alert("회원 정보 수정이 완료되었습니다.");
        navigate("/accounts"); // 유저 정보 페이지 이동
      } else {
        userProgressStore.handleOpenModal("update-user-info");

        console.error("회원 정보 수정 실패:", result.error);
      }
    } catch (error) {
      console.error("요청 처리 중 오류 발생:", error);
    }
  }

  return (
    <Modal
      open={userProgressStore.modalProgress === "update-user-info"}
      onClose={
        userProgressStore.modalProgress === "update-user-info"
          ? userProgressStore.handleCloseModal
          : null
      }
    >
      <form id="signup-form" onSubmit={handleSubmit}>
        <div className="signup-header">
          <h2>회원 정보 수정</h2>
          <button type="button" onClick={userProgressStore.handleCloseModal}>
            ⨉
          </button>
        </div>

        {/* 이메일 입력 */}
        <div className="signup-control">
          <label htmlFor="email">이메일 아이디</label>
          <div className="signup-wrapper">
            <input
              // id="email"
              className="email-input"
              type="email"
              name="email"
              ref={emailInput}
            />
            {formIsInvalid.emailCheck === "" && (
              <button
                type="button"
                onClick={handleEmailCheck}
                className="email-check"
              >
                중복
                <br />
                확인
              </button>
            )}
            {formIsInvalid.emailCheck === "verified" && (
              <button type="button" className="email-verified" disabled>
                확인됨
              </button>
            )}
            {(formIsInvalid.emailCheck === "not-verified" ||
              formIsInvalid.emailCheck === "not-available") && (
              <button
                type="button"
                onClick={handleEmailCheck}
                className="email-not-verified"
              >
                중복
                <br />
                확인
              </button>
            )}
          </div>
          {formIsInvalid.email && (
            <div className="signup-control-error">
              <p>올바른 이메일 양식을 작성해주세요.</p>
            </div>
          )}
          {formIsInvalid.emailCheck === "" && (
            <div className="signup-control-confirm">
              <p>변경하시려면 이메일 입력 후, 중복 확인을 해 주세요.</p>
            </div>
          )}
          {formIsInvalid.emailCheck === "not-verified" && (
            <div className="signup-control-error">
              <p>이메일을 변경하시려면 중복 확인을 해 주세요.</p>
            </div>
          )}
          {formIsInvalid.emailCheck === "not-available" && (
            <div className="signup-control-error">
              <p>이미 사용 중인 이메일입니다.</p>
            </div>
          )}
          {formIsInvalid.emailCheck === "verified" && (
            <div className="signup-control-confirm">
              <p>입력하신 이메일은 사용 가능합니다.</p>
            </div>
          )}
        </div>

        <hr />

        {/* 개인 정보 입력 */}
        <div className="signup-control-row">
          <div className="signup-wrapper">
            <div className="signup-control">
              <label htmlFor="user_name">이름</label>
              <input
                type="text"
                // id="user-name"
                name="user_name"
                defaultValue={userInformation.user_name || ""}
                required
              />
            </div>

            <div className="signup-control">
              <label htmlFor="birth_date">생년월일</label>
              <input
                type="date"
                // id="birth-date"
                name="birth_date"
                defaultValue={
                  userInformation.birth_date ? userInformation.birth_date : ""
                }
                required
              />
            </div>
          </div>

          {/* 주소 입력 */}
          <div className="signup-control-row">
            <div className="signup-wrapper">
              <div className="signup-control">
                <label htmlFor="state">시/도</label>
                <select
                  // id="state"
                  name="state"
                  value={selectedState}
                  onChange={handleStateChange}
                  required
                >
                  <option value="">선택하세요</option>
                  {regions.map((region) => (
                    <option key={region.name} value={region.name}>
                      {region.name}
                    </option>
                  ))}
                </select>
              </div>

              <div className="signup-control">
                <label htmlFor="city">시/군/구</label>
                <select
                  // id="city"
                  name="city"
                  value={selectedCity}
                  onChange={handleCityChange}
                  required
                >
                  <option value="">선택하세요</option>
                  {cities.map((city) => (
                    <option key={city} value={city}>
                      {city}
                    </option>
                  ))}
                </select>
              </div>
            </div>
          </div>
        </div>

        {/* 성별 및 사용자 유형 입력 */}
        <div className="signup-control-row">
          <div className="signup-wrapper">
            <div className="signup-control">
              <label htmlFor="gender">성별</label>
              <select
                id="gender"
                name="gender"
                defaultValue={userInformation.gender}
                required
              >
                <option value="male">남성</option>
                <option value="female">여성</option>
              </select>
            </div>

            <div className="signup-control">
              <label htmlFor="role">사용자 유형</label>
              <select
                id="role"
                name="role"
                defaultValue={userInformation.role}
                required
              >
                <option value="main">주 사용자</option>
                <option value="sub">보조 사용자</option>
              </select>
            </div>
          </div>
        </div>

        <button type="submit" className="signup-btn">
          회원 정보 수정
        </button>
      </form>
    </Modal>
  );
}
