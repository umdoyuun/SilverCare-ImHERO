import "./Widget.css";

export default function Widget({ title, children }) {
  return (
    <div id="widget">
      <div id="widget-top">
        <h3>{title}</h3>
      </div>
      {/* {content && <p>{content}</p>} */}
      <div id="widget-body">{children}</div>
    </div>
  );
}
