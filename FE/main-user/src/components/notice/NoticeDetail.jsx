import React from "react";
import "./Notice.css";

export default function NoticeDetail({ notice, onReply }) {
    if (!notice) {
      return <div className="notice-unselected">알림을 선택하세요</div>;
    }
    
    return (
      <div className="notice-detail">
        <h2>알림 상세</h2>
        <div className="notice-content">
            <p className={"notice-long-text ${notice.type}"}>{notice.text}</p>
            {notice.type === "message" && (
                <button className="reply-button" onClick={onReply}>
                    답장하기
                </button>
            )}
        </div>
      </div>
    );
  }