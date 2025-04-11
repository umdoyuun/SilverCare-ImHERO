import { useContext } from "react";
import Status from "./Status.jsx";
import { UserProgressContext } from "../../store/userProgressStore.jsx";
import "./Navbar.css";

export default function NavBar() {
  const { loginUserInfo, handleLogout } = useContext(UserProgressContext);

  return (
    <aside id="nav-bar">
      <p onDoubleClick={handleLogout} style={{ cursor: "pointer"}}>
        {loginUserInfo.login ? loginUserInfo.userInfo.user_name : "Guest"}
      </p>
      <Status />
    </aside>
  );
}
