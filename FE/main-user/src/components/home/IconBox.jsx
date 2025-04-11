import "./Home.css";

import { useContext } from "react";

import messageIcon from "../../assets/message.png";

import Icon from "./Icon.jsx";
import Info from "./Info.jsx";

import { StoreContext } from "../../store/store.jsx";

export default function IconBox() {
  const store = useContext(StoreContext);

  return (
    <div id="icon-box">
      <Icon
        type="icon"
        imgSrc={messageIcon}
        altSrc="message-icon"
        onClickIcon={store.handleMessageState}
      >
        메시지
      </Icon>
      <Info
        type="info"
      />
    </div>
  );
}
