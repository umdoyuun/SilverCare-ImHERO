import { createContext } from "react";
import { useState } from "react";
import { useContext } from "react";
import { useEffect } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { UserProgressContext } from "./userProgressStore";
import { StoreContext } from "./store";

export const EnvironmentDataContext = createContext({
    environmentData: {
        result: {
            family_id: null,
            reported_at: null,
            temperature: null,
            humidity: null,
            dust_level: null,
            ethanol: null,
        },
    },
    setEnvironmentData: () => {},
})

export default function EnvironmentDataContextProvider({ children }) {
    const { request, loading, error } = useMainHttp();
    const userProgressStore = useContext(UserProgressContext);
    const store = useContext(StoreContext);

    const [environmentData, setEnvironmentData] = useState({
        result: {
            family_id: null,
            reported_at: null,
            temperature: null,
            humidity: null,
            dust_level: null,
            ethanol: null,
        },
    });

    let familyId = userProgressStore.familyInfo?.familyId || "";

    async function handleGetLatestEnvironmentData() {
        if (!familyId) {
        console.error("가족 ID가 없습니다.")
        return {
            success: false,
            error: {
                type: "no_family_id",
                message: "가족 ID가 없습니다.",
            },
        }
        }

        try {
            const response = await request(`${userProgressStore.DEV_API_URL}/status/home/latest/${encodeURIComponent(familyId)}`)
            const resData = response.data

            if (response.success) {
                if (resData.message === "Home status retrieved successfully") {
                // setEnvironmentData({
                //     result: {
                //         family_id: resData.result.family_id,
                //         reported_at: resData.result.reported_at,
                //         temperature: resData.result.temperature,
                //         humidity: resData.result.humidity,
                //         dust_level: resData.result.dust_level.toFixed(2),
                //         ethanol: resData.result.ethanol.toFixed(2),
                //     } 
                // });
                    const newData = {
                        result: {
                            family_id: resData.result.family_id,
                            reported_at: resData.result.reported_at,
                            temperature: resData.result.temperature,
                            humidity: resData.result.humidity,
                            dust_level: resData.result.dust_level.toFixed(2),
                            ethanol: resData.result.ethanol.toFixed(2),
                        }
                    };

                    setEnvironmentData(newData);

                    // 🔥 에탄올 수치 확인 후 긴급 모달 열기
                    if (parseFloat(newData.result.ethanol) > 15) {
                        store.handleEmergencyState();
                    }
                }
            } else {
                console.error("최신 집 내부 정보 조회 실패:", response.error)
                setEnvironmentData({
                    result: {
                        family_id: null,
                        reported_at: null,
                        temperature: null,
                        humidity: null,
                        dust_level: null,
                        ethanol: null,
                    },
                })
                return {
                success: false,
                error: {
                    type: response.error.type,
                    message: response.error.message,
                },
                }
            }
        } catch (error) {
            console.error(error)
            return {
                success: false,
                error: {
                type: "network_error",
                message: "네트워크 오류가 발생했습니다.",
                },
            }
        }
    }

    // 6초마다 데이터 업데이트
    useEffect(() => {
        if (familyId) {
            handleGetLatestEnvironmentData();
            const interval = setInterval(handleGetLatestEnvironmentData, 0.1 * 60 * 1000);
            return () => clearInterval(interval);
        }
    }, [familyId]);

    const ctxValue = {
        loading,
        environmentData,
        setEnvironmentData,
        handleGetLatestEnvironmentData,
    }

    return <EnvironmentDataContext.Provider value={ctxValue}>
        {children}
    </EnvironmentDataContext.Provider>
}