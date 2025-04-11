import React from "react";
import { useState, useEffect, useContext } from "react";
import { useSettingStore } from "../../store/settingStore";
import "./Settings.css";

import wallpaper1 from "../../assets/wallpaper1.png";
import wallpaper2 from "../../assets/wallpaper2.png";

export default function Backgrounds() {
    const { backgrounds, fetchBackgrounds } = useSettingStore();
    const defaultBackgrounds = [
        { index: 0, imageUrl: wallpaper1 },
        { index: 1, imageUrl: wallpaper2 }
    ];

    const [selectedBackground, setSelectedBackground] = useState(localStorage.getItem("background") || wallpaper1);

    useEffect(() => {
        fetchBackgrounds();
    }, []);

    const allBackgrounds = [...defaultBackgrounds, ...backgrounds];

    const changeBackground = (image) => {
        setSelectedBackground(image);
        localStorage.setItem("background", image);
        document.body.style.background = `url(${image})`;
        document.body.style.backgroundSize = "cover";
        document.body.style.backgroundPosition = "center";
    };

    return (
        <div className="background-container">
            {allBackgrounds.map(({ index, imageUrl }) => (
                <div
                    key={index}
                    className="background-item"
                    onClick={() => changeBackground(imageUrl)}
                    style={{ border: selectedBackground === imageUrl ? "3px solid navy" : "none" }}
                >
                    <img src={imageUrl} alt="배경" />
                </div>
            ))}
        </div>
    );
}