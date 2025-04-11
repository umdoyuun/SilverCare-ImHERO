import "./Notification.css";

import { useContext } from "react";

import { UserProgressContext } from "../../../store/userProgressStore.jsx";

import AllNotifications from "./AllNotifications.jsx";
import PublicEmergencyNotifications from "./PublicEmergencyNotifications.jsx";
import Emergency from "../emergency/Emergency.jsx";

import PastNotifications from "./PastNotifications.jsx";
import PastPublicEmergency from "./PastPublicEmergency.jsx";

export default function Notification() {
  const userProgressStore = useContext(UserProgressContext);

  if (!userProgressStore.loginUserInfo.login) {
    return;
  }

  return (
    <div id="notification-main">
      <h2 id="main-container-title">알림</h2>

      <div id="notification-container">
        <AllNotifications />
        <PublicEmergencyNotifications />
        <Emergency />
      </div>

      <PastNotifications />
      <PastPublicEmergency />
    </div>
  );
}
