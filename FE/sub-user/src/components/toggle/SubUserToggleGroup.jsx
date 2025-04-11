import "./Toggle.css";

import { useContext } from "react";

import { UserProgressContext } from "../../store/userProgressStore";

import Toggle from "./Toggle";

import personImage from "../../assets/icons/person.svg";
import sirenImage from "../../assets/icons/siren_question.svg";

export default function ToggleGroup() {
  const userProgressStore = useContext(UserProgressContext);

  return (
    <>
      <div id="toggle-group">
        <Toggle
          name="자동 로그인"
          identifier="auto_login"
          status={userProgressStore.subUserSettings.auto_login ? "good" : "bad"}
          onClickToggle={userProgressStore.handleSubUserSettings}
          imgSrc={personImage}
          altSrc="auto_login"
        ></Toggle>
        <Toggle
          name="긴급 알림 수신"
          identifier="emergency_notification"
          status={
            userProgressStore.subUserSettings.emergency_notification
              ? "good"
              : "bad"
          }
          onClickToggle={userProgressStore.handleSubUserSettings}
          imgSrc={sirenImage}
          altSrc="emergency_notification"
        ></Toggle>
      </div>
    </>
  );
}
