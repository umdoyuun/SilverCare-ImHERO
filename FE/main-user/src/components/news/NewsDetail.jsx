import React, { useRef } from "react";
import "./News.css";

export default function NewsDetail({ news, onBack }) {
    const newsDetailRef = useRef(null);

    const handleTouchStart = (e) => {
        newsDetailRef.current.startY = e.touches[0].clientY;
    };

    const handleTouchMove = (e) => {
        const diff = newsDetailRef.current.startY - e.touches[0].clientY;
        newsDetailRef.current.scrollTop += diff;
        newsDetailRef.current.startY = e.touches[0].clientY;
    };
  
    if (!news) {
      return <div className="news-unselected">뉴스를 선택하세요</div>;
    }
  
    return (
      <div 
        className="news-detail"
        ref={newsDetailRef}
        onTouchStart={handleTouchStart}
        onTouchMove={handleTouchMove}
      >
        <div className="news-content">
          <button className="back-button-detail" onClick={onBack}>←</button>
          <iframe 
            src={news.link} 
            className="news-iframe" 
            title="News Detail"
            sandbox="allow-same-origin allow-scripts allow-popups"
          />
        </div>
      </div>
    );
  }