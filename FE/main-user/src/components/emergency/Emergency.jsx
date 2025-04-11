import React from "react";
import "./Emergency.css";

export default function Emergency() {
    return (
        <div id="emergency-alert">
            <div id="emergency-alert-content">🚨 화재 경보! 🚨</div>
            <p className="emergency-message">
                일산화탄소 수치가 위험 수준입니다!<br />
                즉시 대응하세요!
            </p>
        </div>
    );
}