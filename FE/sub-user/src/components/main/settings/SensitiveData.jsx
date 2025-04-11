import "./Settings.css";

export default function SensitiveData() {
  return (
    <div id="sensitive-data-group">
      <div id="sensitive-data">
        <h3>활동 정보 삭제</h3>
        <div id="sensitive-data-content">
          <p>
            기록된 활동 정보를 DB에서 삭제할 수 있습니다.
            <br />
            삭제된 정보는 복구 불가능합니다.
          </p>
          <button>삭제</button>
        </div>
      </div>
      <div id="sensitive-data">
        <h3>정신 건강 정보 삭제</h3>
        <div id="sensitive-data-content">
          <p>
            기록된 정신 건강 정보 및 보고서를 DB에서 삭제할 수 있습니다.
            <br />
            삭제된 정보는 복구 불가능합니다.
          </p>
          <button>삭제</button>
        </div>
      </div>
    </div>
  );
}
