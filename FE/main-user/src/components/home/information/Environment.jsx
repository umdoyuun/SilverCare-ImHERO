import React from "react";
import { useContext } from "react";
import { EnvironmentDataContext } from "../../../store/environmentData";
import StatusWidget from "./StatusWidget";
import "../Home.css";

import Temperature from "../../../assets/stat-icons/thermostat.svg";
import Humidity from "../../../assets/stat-icons/humidity.svg";
import Airwave from "../../../assets/stat-icons/airwave.svg";
import AirwaveEm from "../../../assets/stat-icons/airwave_em.svg";
import Gas from "../../../assets/stat-icons/heat.svg";
import GasEm from "../../../assets/stat-icons/heat_em.svg";

export default function Environment() {
    const environmentDataStore = useContext(EnvironmentDataContext);

    // 미세 먼지와 일산화탄소 수치 가져오기
    const dustLevel = parseFloat(environmentDataStore.environmentData.result.dust_level ?? 0);
    const ethanolLevel = parseFloat(environmentDataStore.environmentData.result.ethanol ?? 0);

    // 미세 먼지 상태 판단 (40 이상이면 나쁨)
    const dustStatus = dustLevel < 40 ? "good" : "bad";
    const dustIcon = dustStatus === "bad" ? AirwaveEm : Airwave;

    // 일산화탄소 상태 판단 (1.5 이상이면 나쁨)
    const ethanolStatus = ethanolLevel > 0 && ethanolLevel < 15 ? "good" : "bad";
    const ethanolIcon = environmentDataStore.environmentData.result.ethanol === null || environmentDataStore.environmentData.result.ethanol === undefined ? Gas : (ethanolStatus === "bad" ? GasEm : Gas);

    return (
        <div id="environment">
            <div id="environment-info">
                <StatusWidget
                    name="실내 온도"
                    imgSrc={Temperature}
                    altSrc="temperature"
                    status={`${environmentDataStore.environmentData.result.temperature} °C`}
                ></StatusWidget>
                <StatusWidget
                    name="실내 습도"
                    imgSrc={Humidity}
                    altSrc="humidity"
                    status={`${environmentDataStore.environmentData.result.humidity} %`}
                ></StatusWidget>
                <StatusWidget
                    name="미세 먼지"
                    imgSrc={dustIcon}
                    altSrc="finedust"
                    status={`${dustLevel.toFixed(2)} ㎍/㎥`}
                ></StatusWidget>
                <StatusWidget
                    name="일산화탄소"
                    imgSrc={ethanolIcon}
                    altSrc="ethanol"
                    status={`${ethanolLevel.toFixed(2)} %`}
                >
                    {environmentDataStore.ethanol > 0 ? "가스 감지됨" : "가스 정상"}
                </StatusWidget>
            </div>
        </div>
    )
}