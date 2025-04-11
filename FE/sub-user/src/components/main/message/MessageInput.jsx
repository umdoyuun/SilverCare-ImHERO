import "./Message.css";

import { useRef, useState, useContext } from "react";
import { MessageContext } from "../../../store/messageStore";

import fileIcon from "../../../assets/feature/inbox-2-line.png";
import { set } from "date-fns";

export default function MessageInput() {
  const messageStore = useContext(MessageContext);

  const [image, setImage] = useState(null);
  const [preview, setPreview] = useState(null);

  const inputMessage = useRef("");

  async function handleImageChange(event) {
    const file = event.target.files[0]; // 선택한 파일 가져오기
    // console.log(file);
    if (!file) return;

    setImage(file); // file 객체만 저장

    const imageUrl = URL.createObjectURL(file);
    setPreview(imageUrl);
  }

  async function handleSubmit(event) {
    event.preventDefault();

    let imageUrl = null;

    try {
      if (image) {
        const allowedExtensions = ["jpg", "jpeg", "png", "gif", "webp"];
        const fileExtension = image.name.split(".").pop().toLowerCase();

        if (!allowedExtensions.includes(fileExtension)) {
          console.error("지원하지 않는 파일 형식입니다.");
          return;
        }

        const formData = new FormData();
        formData.append("file", image);

        const response = await messageStore.insertPhotoFile(formData);

        if (!response.success) {
          console.error("이미지 업로드 실패:", response.error);
          return; // 실패 시 종료
        }

        imageUrl = response.data.file_path;
        // console.log("이미지 업로드 성공:", imageUrl);
      }

      const content = inputMessage.current.value.trim(); // 공백만 있는 메시지 방지

      if (!imageUrl && !content) {
        console.warn("메시지 내용 또는 이미지가 필요합니다.");
        return; // 빈 메시지 전송 방지
      }

      const sendResponse = await messageStore.handleSendMessage(
        content,
        imageUrl
      );
      // console.log(sendResponse);

      if (!sendResponse.success) {
        console.error("메시지 전송 실패:", sendResponse.error);
        return; // 실패 시 종료
      }

      // 성공 시 초기화
      setImage(null);
      setPreview(null);
      inputMessage.current.value = "";
    } catch (error) {
      console.error("handleSubmit에서 오류 발생:", error);
    }
  }

  if (!messageStore.messagePerson) {
    return;
  }

  return (
    <div id="message-form-group">
      <form onSubmit={handleSubmit}>
        {/* 사진 첨부 기능 */}
        <div id="message-file-group">
          <label htmlFor="message-file">
            {preview ? (
              <img
                src={preview}
                alt="파일 첨부"
                style={{
                  maxWidth: "90%",
                  maxHeight: "90%",
                  width: "auto",
                  height: "auto",
                }}
              />
            ) : (
              <img src={fileIcon} alt="파일 첨부" width="30" />
            )}
          </label>
          <input
            id="message-file"
            type="file"
            accept="image/*"
            onChange={handleImageChange}
            style={{ display: "none" }} // 숨겨진 파일 선택 버튼
          />
        </div>

        {/* 미리보기 이미지 */}
        {/* {image && (
          <span id="image-preview">
            <img src={preview} alt="미리보기" width="100" />
          </span>
        )} */}

        <div id="message-input-group">
          <input
            id="message-input"
            type="text"
            name="message"
            ref={inputMessage}
          />
          <button type="submit">전송</button>
        </div>
      </form>
    </div>
  );
}
