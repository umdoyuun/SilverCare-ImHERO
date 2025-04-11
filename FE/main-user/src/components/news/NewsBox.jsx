import React, { useContext, useRef, useState } from "react";
import { NewsStoreContext } from "../../store/newsStore";
import NewsDetail from "./NewsDetail";
import defaultImage from "../../assets/icons/hero.png";
import "./News.css";

const categoryMap = {
  business: "경제",
  entertainment: "엔터",
  environment: "환경",
  health: "건강",
  politics: "정치",
  science: "과학",
  sports: "스포츠",
  technology: "기술"
};

export default function NewsBoxPage({ category, newsData, onBack }) {
  const { selectedNews, selectNews, clearSelectedNews } = useContext(NewsStoreContext);
  const newsListRef = useRef(null);
  const isDragging = useRef(false);
  const startY = useRef(0);
  const scrollTop = useRef(0);

  if (selectedNews) {
    return <NewsDetail news={selectedNews} onBack={clearSelectedNews} />
  }

  const handleMouseDown = (e) => {
    isDragging.current = true;
    startY.current = e.clientY;
    scrollTop.current = newsListRef.current.scrollTop;
  };

  const handleMouseMove = (e) => {
    if (!isDragging.current) return;
    const deltaY = e.clientY - startY.current;
    newsListRef.current.scrollTop = scrollTop.current - deltaY;
  };

  const handleMouseUp = () => {
    isDragging.current = false;
  };

  return (
    <div className="news-box-page">
      <div className="news-header"> 
        <button className="back-button" onClick={onBack}>←</button>
        <h2>{categoryMap[category] || category} 뉴스</h2>
      </div>

      <div 
        className="news-list"
        ref={newsListRef}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
      >
        {newsData.length > 0 ? (
          newsData.map((news) => (
            <div key={news.id} className="news-box" onClick={() => selectNews(news)}>
              <img 
                src={news.image_url || defaultImage} 
                alt="News" 
                className="news-image" 
              />
              <div className="news-info">
                <h3 className="news-title">{news.title}</h3>
                <p className="news-meta">
                  {new Date(news.pub_date).toLocaleDateString()}
                </p>
              </div>
            </div>
          ))
        ) : (
          <p className="news-unselected">해당 카테고리 뉴스가 없습니다.</p>
        )}
      </div>
    </div>
  );
}