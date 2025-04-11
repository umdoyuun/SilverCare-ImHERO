import React from "react";
import { useEffect, useContext } from "react";
import { StoreContext } from "../../store/store";
import { useSettingStore } from "../../store/settingStore";
import { useUserProgressStore } from "../../store/userProgressStore";
import "./Message.css";

export default function Message({ text, sender, time, imageUrl }) {
    const { familyInfo } = useUserProgressStore();
    const { backgrounds, fetchBackgrounds, addBackground } = useSettingStore();
    const { handleMiniModal } = useContext(StoreContext);

    // useEffect(() => {
    //     fetchBackgrounds();
    // }, []);

    const handleAddToBackground = () => {
        // if (!familyInfo.familyId || !imageUrl) return;
        
        // const isDuplicate = backgrounds.some(bg => bg.imageUrl === imageUrl);
        // if (isDuplicate) {
        //     handleMiniModal("⚠️ 이미 다운로드된 배경화면입니다.");
        //     return;
        // }

        addBackground(imageUrl);
        handleMiniModal("✅ 배경화면이 성공적으로 다운로드되었습니다!");
    }

    return (
        <div className={`message-wrapper ${sender === "me" ? "my-message" : "other-message"}`}>
            {sender === "me" && <span className="message-time left">{time}</span>}
            <div className={`message ${sender === "me" ? "mainText" : "subText"}`}>
            {imageUrl && (
                <div className="image-container">
                    <img src={imageUrl} alt="첨부 이미지" className="message-image" />
                    <button className="add-background-button" onClick={handleAddToBackground}>
                        ➕
                    </button>
                </div>
            )}
                <p className="message-text">{text}</p>
            </div>
            {sender === "other" && <span className="message-time right">{time}</span>}
        </div>
    );
}