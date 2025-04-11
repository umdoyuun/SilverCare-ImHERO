import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { loadEnvironments } from "./store/environmentsStore.jsx";
import "./index.css";
import App from "./App.jsx";

import StoreContextProvider from "./store/store.jsx";
import UserProgressContextProvider from "./store/userProgressStore.jsx";
import SettingStoreContextProvider from "./store/settingStore.jsx";
import MessageProvider from "./store/messageStore.jsx";
import NotificationProvider from "./store/notificationStore.jsx";
import DisasterStoreContextProvider from "./store/disasterStore.jsx";
import EnvironmentDataContextProvider from "./store/environmentData.jsx";
import WeatherStoreContextProvider from "./store/weatherStore.jsx";
import NewsStoreContextProvider from "./store/newsStore.jsx";

const startApp = async () => {
  await loadEnvironments();

  createRoot(document.getElementById("root")).render(
    <StrictMode>
      <StoreContextProvider>
        <UserProgressContextProvider>
          <SettingStoreContextProvider>
            <MessageProvider>
              <DisasterStoreContextProvider>
                <NotificationProvider>
                  <EnvironmentDataContextProvider>
                    <WeatherStoreContextProvider>
                      <NewsStoreContextProvider>
                        <App />
                      </NewsStoreContextProvider>
                    </WeatherStoreContextProvider>  
                  </EnvironmentDataContextProvider>
                </NotificationProvider> 
              </DisasterStoreContextProvider>
            </MessageProvider>
          </SettingStoreContextProvider>
        </UserProgressContextProvider>
      </StoreContextProvider>
    </StrictMode>
  );
};

startApp();