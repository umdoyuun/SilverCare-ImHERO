import React from "react";
import { useState, useEffect } from "react";
import "./Notice.css";

export default function NoticeBox({ notice, onClick }) {
    const [isRead, setIsRead] = useState(notice.is_read);

    useEffect(() => {
        setIsRead(notice.is_read);
    }, [notice.is_read]);

    const truncatedText = notice.text.length > 26 
        ? notice.text.slice(0, 26) + "..."
        : notice.text;

    return (
        <div className={`notice-box ${notice.notification_grade} ${isRead ? "read" : ""}`} onClick={() => {
            onClick(notice)
        }}>
            <p className="notice-text">{truncatedText}</p>
            <span className="notice-time">{notice.time}</span>
        </div>
    );
}