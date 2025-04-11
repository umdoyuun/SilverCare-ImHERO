import "./PageContainer.css";

export default function PageContainer({ title, children }) {
  return (
    <div id="main-page-container">
      <div id="main-page-container-top">
        <h3>{title}</h3>
      </div>
      <div id="main-page-container-body">{children}</div>
    </div>
  );
}
