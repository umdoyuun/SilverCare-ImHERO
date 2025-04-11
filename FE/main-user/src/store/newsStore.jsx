import { useState, createContext, useEffect, useContext } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { UserProgressContext } from "./userProgressStore";

export const NewsStoreContext = createContext({
  newsData: {},
  isLoading: false,
  selectedNews: null,
  fetchNewsData: () => {},
  selectNews: () => {},
  clearSelectedNews: () => {},
});

export default function NewsStoreContextProvider({ children }) {
  const { request } = useMainHttp();
  const userProgressStore = useContext(UserProgressContext);

  const [newsData, setNewsData] = useState({});
  const [isLoading, setIsLoading] = useState(false);
  const [selectedNews, setSelectedNews] = useState(null);

  async function fetchNewsData() {
    setIsLoading(true);

    try {
      if(!userProgressStore.loginUserInfo.login) return;

      const today = new Date();
      today.setDate(today.getDate() - 1);
      const formattedDate = today.toISOString().split("T")[0];
      
      const response = await request(`${userProgressStore.DEV_API_URL}/tools/news?when=${formattedDate}`);
      const resData = response.data;
      
      if (response.success) {
        if (resData.message === "News retrieved successfully") {
          setNewsData(resData.result);
        }
      } else {
        console.error("뉴스 데이터를 불러오는 중 오류 발생:", resData.error);
        setNewsData({});
      }
    } catch (error) {
      console.error("API 요청 실패:", error);
      setNewsData({});
    } finally {
      setIsLoading(false);
    }
  };

  function selectNews(news) {
    setSelectedNews(news);
  }

  function clearSelectedNews() {
    setSelectedNews(null);
  }

  useEffect(() => {
    if (userProgressStore.loginUserInfo.login) {
      console.log("🔄 로그인 완료됨, 뉴스 데이터 가져오기 실행");
      fetchNewsData();
    }
  }, [userProgressStore.loginUserInfo.login]);

  const ctxValue = {
    newsData,
    isLoading,
    selectNews,
    fetchNewsData,
    selectedNews,
    clearSelectedNews,
  };

  return (
    <NewsStoreContext.Provider value={ctxValue}>
      {children}
    </NewsStoreContext.Provider>
  );
}