import React, { useContext, useState } from "react";
import { NewsStoreContext } from "../../store/newsStore";
import NewsBox from "./NewsBox";
import "./News.css";

const defaultCategories = {
  "경제": "business", 
  "엔터": "entertainment", 
  "환경": "environment", 
  "건강": "health",
  "정치": "politics", 
  "과학": "science", 
  "스포츠": "sports", 
  "기술": "technology"
};

export default function News({ onReply }) {
  const { newsData, isLoading } = useContext(NewsStoreContext);
  const [selectedCategory, setSelectedCategory] = useState(null);

  const categories = Object.keys(newsData).length > 0 ? Object.keys(newsData) : Object.values(defaultCategories);

  // 카테고리 클릭 시 해당 뉴스 페이지로 전환
  if (selectedCategory) {
    return (
    <NewsBox 
      category={selectedCategory} 
      newsData={newsData[selectedCategory] || []}
      onBack={() => setSelectedCategory(null)} 
      />
    );
  }

  return (
    <div className="news-category-container">
      {isLoading ? (
        <p>뉴스 데이터를 불러오는 중...</p>
      ) : (
        <div className="news-category-buttons">
          {Object.entries(defaultCategories).map(([kor, eng]) => (
            <button
              key={eng}
              className="news-category-button"
              onClick={() => setSelectedCategory(eng)}
            >
              {kor}
            </button>
          ))}
        </div>
      )}
    </div>
  );
}