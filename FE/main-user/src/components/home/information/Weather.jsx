import React from "react";
import { useContext } from "react";
import { WeatherStoreContext } from "../../../store/weatherStore";
import "../Home.css";

import sunny from "../../../assets/weather/sunny.png";
import littleCloud from "../../../assets/weather/little_cloud.png";
import hellCloud from "../../../assets/weather/hell_cloud.png";
import foggy from "../../../assets/weather/foggy.png";
import rain from "../../../assets/weather/rain.png";
import rainow from "../../../assets/weather/rainow.png";
import snow from "../../../assets/weather/snow.png";
import shower from "../../../assets/weather/shower.png";

export default function Weather() {
    const { weatherData, isLoading } = useContext(WeatherStoreContext);

    const skyIcons = {
        "맑음": sunny,
        "구름조금": littleCloud,
        "구름많음": hellCloud,
        "흐림": foggy,
    }

    const rainIcons = {
        "비": rain,
        "비/눈": rainow,
        "눈": snow,
        "소나기": shower,
    }

    let weatherIcon = skyIcons[weatherData.sky] || skyIcons["맑음"];

    if (weatherData.sky !== "없음") {
        weatherIcon = rainIcons[weatherData.sky] || weatherIcon;
    }


    return (
        <div id="weather" style={{ height: "100%" }}>
            <div id="weather-title">날씨</div>
            <div id="weather-info">
                <img src={weatherIcon} alt="weather-icon"/>
                <div id="weather-detail">
                    <div id="weather-temp">{`${weatherData.temperature}`}</div>
                    <div id="weather-location">{`${weatherData.address}`}</div>
                </div>
            </div>
        </div>

    )
}