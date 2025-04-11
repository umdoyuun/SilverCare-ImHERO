import "./Message.css";

import { useContext } from "react";

import { UserProgressContext } from "../../../store/userProgressStore";
import { MessageContext } from "../../../store/messageStore";

export default function SelectMessageRoom() {
  const userProgressStore = useContext(UserProgressContext);
  const messageStore = useContext(MessageContext);

  if (
    userProgressStore.loginUserInfo.login &&
    userProgressStore.loginUserInfo.userInfo.role === "main"
  ) {
    return;
  }

  const registeredFamilyInfo = userProgressStore.memberInfo.registerData;

  return (
    <>
      {registeredFamilyInfo && (
        <div className="select-message-room">
          {registeredFamilyInfo.map((family) => {
            return (
              <button
                className={
                  messageStore.messagePerson === family.main_user_id
                    ? "selected"
                    : "unselected"
                }
                key={family.id}
                onClick={() =>
                  messageStore.setMessagePerson(family.main_user_id)
                }
              >
                {family.family_name}
              </button>
            );
          })}
        </div>
      )}
    </>
  );
}
