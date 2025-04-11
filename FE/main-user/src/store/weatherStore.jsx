import { useState, createContext, useEffect, useContext } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { UserProgressContext } from "./userProgressStore";

export const WeatherStoreContext = createContext({
    weatherData: {},
    isLoading: false,
    fetchWeatherData: () => {},
});

export default function WeatherStoreContextProvider({ children }) {
    const { request } = useMainHttp();
    const userProgressStore = useContext(UserProgressContext);

    const [weatherData, setWeatherData] = useState({});
    const [isLoading, setIsLoading] = useState(false);

    let userId = userProgressStore.loginUserInfo.userInfo?.id || "";

    async function fetchWeatherData() {
        setIsLoading(true);

        try {
            const response = await request(`${userProgressStore.DEV_API_URL}/tools/weather/${encodeURIComponent(userId)}`);
            const resData = response.data;

            if (response.success && resData.message === "Weather retrieved successfully") {
            console.log("날씨 데이터 저장:", resData.result);
            setWeatherData(resData.result);
            } else {
            console.error("날씨 데이터를 불러오는 중 오류 발생:", resData.error);
            setWeatherData({});
            }
        } catch (error) {
            console.error("API 요청 실패:", error);
            setWeatherData({});
        } finally {
            setIsLoading(false);
            console.log("날씨 API 요청 종료");
        }
    }

    useEffect(() => {
        if (userId) {
            fetchWeatherData();
            const interval = setInterval(fetchWeatherData, 60 * 60 * 1000);
            return () => clearInterval(interval);
        }
    }, [userId]);

    const ctxValue = {
        weatherData,
        isLoading,
        fetchWeatherData,
    };

    return (
        <WeatherStoreContext.Provider value={ctxValue}>
            {children}
        </WeatherStoreContext.Provider>
    );
}