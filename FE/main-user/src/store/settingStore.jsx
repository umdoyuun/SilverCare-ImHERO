import { useState, createContext, useEffect, useContext } from "react";
import { useMainHttp } from "../hooks/useMainHttp";
import { UserProgressContext } from "./userProgressStore";

export const SettingStoreContext = createContext({
    backgrounds: [],
    alertState: true,
    cameraState: true,
    driveState: true,
    micState: true,
    fetchBackgrounds: () => {},
    addBackground: () => {},
    toggleFeature: () => {},
    fetchSettings: () => {},
  });
  
  export function useSettingStore() {
    return useContext(SettingStoreContext);
  }

  export default function SettingStoreContextProvider({ children }) {
    const { request } = useMainHttp();
    const userProgressStore = useContext(UserProgressContext);
    const [backgrounds, setBackgrounds] = useState([]);

    const [settings, setSettings] = useState({
        alertState: false,
        cameraState: false,
        driveState: false,
        micState: false,
    });

    const familyId = userProgressStore.familyInfo?.familyId || "";

    async function fetchSettings() {
        if (!familyId) return;

        try {
            const response = await request(
                `${userProgressStore.DEV_API_URL}/tools/settings/${familyId}`,
                "GET"
            );
            
            const resData = response.data;
            
            if (response.success && resData.message === "Settings retrieved successfully") {
                setSettings({
                    alertState: resData.result.is_alarm_enabled,
                    cameraState: resData.result.is_camera_enabled,
                    driveState: resData.result.is_driving_enabled,
                    micState: resData.result.is_microphone_enabled,
                });
            }
        } catch (error) {
            console.error("❌ 설정 값 불러오기 실패:", error);
        }
    }

    async function camcarToggle(featureKey) {
        try {
            const updates = {};

            if (featureKey === "cameraState") {
                updates.is_camera_enabled = !settings.cameraState;
                updates.is_driving_enabled = settings.driveState;
            } else if (featureKey === "driveState") {
                updates.is_camera_enabled = settings.cameraState;
                updates.is_driving_enabled = !settings.driveState;
            }

            // 변경된 내용이 없다면 요청 보내지 않음
            if (Object.keys(updates).length === 0) {
                console.log("⚠️ 변경된 사항이 없어 요청을 보내지 않습니다.");
                return;
            }
            
            const response = await request(`http://70.12.247.214:8001/SettingValue`, "POST", updates);
            
            if (response.success) {
                console.log("카메라/자동차 상태 변경 성공:", response.data);
            } else {
                console.error("❌ 카메라/자동차 상태 변경 실패:", response.error);
                setSettings((prev) => ({
                    ...prev,                              
                    cameraState: settings.cameraState,
                    driveState: settings.driveState,
                }));
            }
        } catch (error) {
            console.error("❌ 네트워크 오류 발생:", error);
            setSettings((prev) => ({
                ...prev,
                cameraState: settings.cameraState,
                driveState: settings.driveState,
            }));
        }
    }

    async function audioToggle() {
        try {
            const updatedMicState = !settings.micState;
            setSettings((prev) => ({ ...prev, micState: updatedMicState }));

            const response = await request(`http://70.12.247.214:8001/bluetooth/speaker/toggle`, "POST");

            const resData = response.data;

            if (response.success && resData?.is_microphone_enabled !== undefined) {
                setSettings((prev) => ({
                    ...prev,
                    micState: response.data.is_microphone_enabled,
                }));
                console.log("마이크 상태 변경 성공:", response.data.is_microphone_enabled);
            } else {
                console.error("❌ 마이크 상태 변경 실패:", response.error);
                setSettings((prev) => ({ ...prev, micState: !updatedMicState }));
            }
        } catch (error) {
            console.error("❌ 네트워크 오류 발생:", error);
            setSettings((prev) => ({ ...prev, micState: !prev.micState }));
        }
    }

    async function toggleFeature(featureKey) {
        if (!familyId) return;

        const updatedSettings = {
            ...settings,
            [featureKey]: !settings[featureKey],
        };

        setSettings(updatedSettings);

        try {
            const response = await request(
                `${userProgressStore.DEV_API_URL}/tools/settings/${familyId}`,
                "PATCH",
                { 
                    is_alarm_enabled: updatedSettings.alertState,
                    is_camera_enabled: updatedSettings.cameraState,
                    is_driving_enabled: updatedSettings.driveState,
                }
            );

            console.log("📡 PATCH 요청 결과:", response);

            if (!response.success) {
                setSettings(settings);
                console.error(`❌ ${featureKey} 상태 변경 실패:`, response.error);
            } else {
                console.log(`${featureKey} 상태 변경 성공:`, updatedSettings);
            }
        } catch (error) {
            console.error(`❌ ${featureKey} 상태 변경 중 오류 발생:`, error);
        }
    }

    async function fetchBackgrounds() {
        if (!familyId) return;

        try {
            const response = await request(
                `${userProgressStore.DEV_API_URL}/tools/background/${familyId}?uploader=mine`,
                "GET"
            );

            const resData = response.data;

            if (response.success) {
                setBackgrounds(resData.result.map(bg => ({
                    index: bg.id,
                    imageUrl: bg.image_url
                })));
            } else {
                console.warn("⚠️ 배경화면 목록이 비어 있습니다.");
                setBackgrounds([]);
            }
        } catch (error) {
            console.error("❌ 배경화면 목록 불러오기 실패:", error);
        }
    }

    async function addBackground(imageUrl) {
        if (!imageUrl || !familyId) return;

        try {
            await fetchBackgrounds(); // 🔄 최신 배경 목록 가져오기

            // const isDuplicate = backgrounds.some(bg => bg.imageUrl === imageUrl);
            // if (isDuplicate) {
            //     handleMiniModal("⚠️ 이미 다운로드된 배경화면입니다.");
            //     return;
            // }

            const response = await request(`${userProgressStore.DEV_API_URL}/tools/background`, "POST", {
                family_id: familyId,
                image_url: imageUrl,
            });

            const resData = response.data;

            if (response.success && resData.result) {
                alert("📸 이미지가 저장되었습니다!");
            } else {
                console.error("❌ 배경 추가 실패:", response.error);
            }
        } catch (error) {
            console.error("❌ 네트워크 오류:", error);
        }
    }

    useEffect(() => {
        if (familyId) {
            fetchSettings();
        }
    }, [familyId]);
  
    const ctxValue = {
        backgrounds,
        ...settings,
        fetchBackgrounds,
        addBackground,
        toggleFeature,
        fetchSettings,
        camcarToggle,
        audioToggle,
    };
  
    return (
        <SettingStoreContext.Provider value={ctxValue}>
            {children}
        </SettingStoreContext.Provider>
    );
  }  