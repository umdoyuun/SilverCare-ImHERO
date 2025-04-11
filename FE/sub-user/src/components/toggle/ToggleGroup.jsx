import "./Toggle.css";

import { useContext } from "react";

import { UserProgressContext } from "../../store/userProgressStore";
import { HomeStatusContext } from "../../store/homeStatusStore";
import { HealthContext } from "../../store/healthStore";

import Toggle from "./Toggle";
import StatusToggle from "./StatusToggle";

import notificationOnImage from "../../assets/feature/notification-4-line.png";
import notificationOffImage from "../../assets/feature/notification-off-line.png";
import carImage from "../../assets/feature/steering-2-line.png";
import cameraImage from "../../assets/feature/camera-line.png";
import cameraOffImage from "../../assets/feature/camera-off-line.png";
import micOnImage from "../../assets/feature/mic-line.png";
import micOffImage from "../../assets/feature/mic-off-line.png";
import thermometerImage from "../../assets/feature/temp-hot-line.png";
import heatImage from "../../assets/feature/blaze-line.png";
import airImage from "../../assets/feature/sparkling-line.png";
import fineAirImage from "../../assets/feature/mist-line.png";
import humidityImage from "../../assets/feature/water-percent-line.png";
import heartImage from "../../assets/feature/heart-pulse-line.png";

export default function ToggleGroup() {
  const userProgressStore = useContext(UserProgressContext);
  const homeStatusStore = useContext(HomeStatusContext);
  const healthStore = useContext(HealthContext);

  const dustLevel =
    homeStatusStore.homeStatus.length > 0
      ? homeStatusStore.homeStatus[0].dust_level.toFixed(0)
      : null;
  const ethanol =
    homeStatusStore.homeStatus.length > 0
      ? homeStatusStore.homeStatus[0].ethanol.toFixed(3)
      : null;
  const heartRate =
    (healthStore.healthStatus && healthStore.healthStatus[0]?.heart_rate) ||
    null;

  // console.log(dustLevel, ethanol, heartRate);

  return (
    <div id="toggle-group-container">
      <div id="toggle-group">
        <Toggle
          name="알림"
          identifier="notification"
          status={
            homeStatusStore.deviceStatus.is_alarm_enabled ? "good" : "bad"
          }
          imgSrc={
            homeStatusStore.deviceStatus.is_alarm_enabled
              ? notificationOnImage
              : notificationOffImage
          }
          altSrc="notification"
        ></Toggle>
        <Toggle
          name="카메라"
          identifier="camera"
          status={
            homeStatusStore.deviceStatus.is_camera_enabled ? "good" : "bad"
          }
          imgSrc={
            homeStatusStore.deviceStatus.is_camera_enabled
              ? cameraImage
              : cameraOffImage
          }
          altSrc="camera"
        ></Toggle>
        <Toggle
          name="마이크"
          identifier="microphone"
          status={
            homeStatusStore.deviceStatus.is_microphone_enabled ? "good" : "bad"
          }
          imgSrc={
            homeStatusStore.deviceStatus.is_microphone_enabled
              ? micOnImage
              : micOffImage
          }
          altSrc="microphone"
        ></Toggle>
        <Toggle
          name="주행"
          identifier="car"
          status={
            homeStatusStore.deviceStatus.is_driving_enabled ? "good" : "bad"
          }
          imgSrc={carImage}
          altSrc="car"
        ></Toggle>
      </div>
      <div id="toggle-group">
        <StatusToggle
          name="온도"
          imgSrc={thermometerImage}
          altSrc="temperature"
          statusLevel={
            homeStatusStore.homeStatus.length > 0 &&
            homeStatusStore.homeStatus[0].temperature &&
            18 < homeStatusStore.homeStatus[0].temperature &&
            homeStatusStore.homeStatus[0].temperature < 26
              ? "good"
              : "bad"
          }
          status={`${
            homeStatusStore.homeStatus.length > 0
              ? `${homeStatusStore.homeStatus[0].temperature}`
              : "-"
          }`}
          symbol="℃"
        />
        <StatusToggle
          name="습도"
          imgSrc={humidityImage}
          altSrc="humidity"
          statusLevel={
            homeStatusStore.homeStatus.length > 0 &&
            homeStatusStore.homeStatus[0].humidity &&
            40 < homeStatusStore.homeStatus[0].humidity &&
            homeStatusStore.homeStatus[0].humidity < 70
              ? "good"
              : "bad"
          }
          status={`${
            homeStatusStore.homeStatus.length > 0
              ? `${homeStatusStore.homeStatus[0].humidity}`
              : "-"
          }`}
          symbol="%"
        />
        <StatusToggle
          name="미세먼지"
          imgSrc={airImage}
          altSrc="dust"
          statusLevel={
            homeStatusStore.homeStatus.length > 0 &&
            homeStatusStore.homeStatus[0].dust_level &&
            homeStatusStore.homeStatus[0].dust_level < 40
              ? "good"
              : "bad"
          }
          status={`${dustLevel ? `${dustLevel}` : "-"}`}
          symbol="㎍/㎥"
        />
        <StatusToggle
          name="초미세먼지"
          imgSrc={fineAirImage}
          altSrc="finedust"
          statusLevel={
            homeStatusStore.homeStatus.length > 0 &&
            homeStatusStore.homeStatus[0].others?.ultrafinedust &&
            homeStatusStore.homeStatus[0].others?.ultrafinedust < 30
              ? "good"
              : "bad"
          }
          status={`${
            homeStatusStore.homeStatus.length > 0 &&
            homeStatusStore.homeStatus[0].others.ultrafinedust
              ? `${homeStatusStore.homeStatus[0].others.ultrafinedust}`
              : "-"
          }`}
          symbol="㎍/㎥"
        />
        <StatusToggle
          name="일산화탄소"
          imgSrc={heatImage}
          altSrc="gas"
          statusLevel={
            homeStatusStore.homeStatus.length > 0 &&
            homeStatusStore.homeStatus[0].ethanol > 0 &&
            homeStatusStore.homeStatus[0].ethanol < 1.5
              ? "good"
              : "bad"
          }
          status={`${ethanol ? `${ethanol}` : "-"}`}
          symbol="%"
        />
        <StatusToggle
          name="심박"
          imgSrc={heartImage}
          altSrc="heartrate"
          statusLevel={60 < heartRate || heartRate < 120 ? "good" : "bad"}
          status={`${heartRate ? `${heartRate}` : "-"}`}
          symbol="bpm"
        />
      </div>
    </div>
  );
}
