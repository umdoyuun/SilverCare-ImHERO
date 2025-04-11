import { useContext } from "react";
import { StoreContext } from "../../store/store.jsx";

import Modal from "./Modal.jsx";
import Emergency from "../emergency/Emergency.jsx";
import EmergencyModal from "./EmergencyModal.jsx";
import MessageModal from "./MessageModal.jsx";
import NoticeModal from "./NoticeModal.jsx";
import NewsModal from "./NewsModal.jsx";
import CalendarModal from "./CalendarModal.jsx";
import SettingModal from "./SettingModal.jsx";

export default function ModalPage() {
  const store = useContext(StoreContext);

  return (
    <>
      <Modal open={store.openMessageState} onClose={store.handleModalClose}>
        {store.openMessageState && (
          <MessageModal
            title="메세지"
            message="message"
            onCloseConfirm={store.handleModalClose}
          />
        )}
      </Modal>
      <Modal open={store.openEmergencyState} onClose={store.handleModalClose}>
        {store.openEmergencyState && (
          <EmergencyModal
              title="긴급"
              onCloseConfirm={store.handleModalClose}>
              <Emergency />
          `</EmergencyModal>
        )}
      </Modal>
      <Modal open={store.openNotificationState} onClose={store.handleModalClose}>
        {store.openNotificationState && (
          <NoticeModal
            title="알림"
            message="Check notifications"
            onCloseConfirm={store.handleModalClose}
          />
        )}
      </Modal>
      <Modal open={store.openNewsState} onClose={store.handleModalClose}>
        {store.openNewsState && (
          <NewsModal
            title="뉴스"
            message="Check News"
            onCloseConfirm={store.handleModalClose}
          />
        )}
      </Modal>
      <Modal open={store.openCalendarState} onClose={store.handleModalClose}>
        {store.openCalendarState && (
          <CalendarModal
            title="달력"
            message="Check your schedule"
            onCloseConfirm={store.handleModalClose}
          />
        )}
      </Modal>
      <Modal open={store.openSettingState} onClose={store.handleModalClose}>
        {store.openSettingState && (
          <SettingModal
            title="설정"
            onCloseConfirm={store.handleModalClose}>
          </SettingModal>
        )}
      </Modal>
    </>
  );
}
