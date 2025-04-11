import "./Settings.css";

export default function Settings({type, children}) {
    return (
      <div id={type}>
        {type === "box" && <div className="box-name">{children}</div>}
      </div>
    ); 
  }