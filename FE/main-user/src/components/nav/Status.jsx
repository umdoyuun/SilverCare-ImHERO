import { useContext } from "react";
import { StoreContext } from "../../store/store.jsx";
import { useNotificationStore } from "../../store/notificationStore.jsx";

import StatusIcon from "./StatusIcon.jsx";
import newsIcon from "../../assets/aside/side-news.png";
import batteryIconCharge from "../../assets/aside/side-battery-charge.png";
import notificationIcon from "../../assets/aside/side-notification.png";
import isNotificationIcon from "../../assets/aside/side-notification-new.png";
import settingIcon from "../../assets/aside/side-setting.png";
import calendarIcon from "../../assets/aside/side-calendar.png";

export default function Status() {
  const store = useContext(StoreContext);
  const { notifications } = useNotificationStore();

  // 읽지 않은 알림이 있는지 확인
  const hasUnreadNotifications = notifications.some(notice => !notice.is_read);

  return (
    <div id="status-bar">
      <StatusIcon
        imgSrc={hasUnreadNotifications ? isNotificationIcon : notificationIcon}
        altSrc="notification-icon"
        status={store.openNotificationState}
        onClickIcon={store.handleNotificationState}
      />
      <StatusIcon
        imgSrc={newsIcon}
        altSrc="news-icon"
        status={store.openNewsState}
        onClickIcon={store.handleNewsState}
      />
      <StatusIcon
        imgSrc={calendarIcon}
        altSrc="calendar-icon"
        status={store.openCalendarState}
        onClickIcon={store.handleCalendarState}
      />
      <StatusIcon imgSrc={batteryIconCharge} altSrc="battery-icon-charge" className="no-border" />
      <StatusIcon
        imgSrc={settingIcon}
        altSrc="setting-icon"
        status={store.openSettingState}
        onClickIcon={store.handleSettingState}
      />
    </div>
  );
}
