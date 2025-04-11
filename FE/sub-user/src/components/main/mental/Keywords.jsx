import "./Mental.css";

import { useContext } from "react";

import { HealthContext } from "../../../store/healthStore";

// import Widget from "../../widget/Widget";

export default function Keywords() {
  const healthStore = useContext(HealthContext);

  const mainUserName = "박순자123";

  return (
    <div id="keywords">
      <div id="keywords-top">
        <h3>{`${mainUserName}님의 대화 키워드`}</h3>
      </div>

      <div id="keywords-body">
        {healthStore.keywords.map((content, index) => {
          return (
            <span
              key={index}
              id="keyword-box"
              style={{
                backgroundColor:
                  healthStore.keywordColors[
                    index % healthStore.keywordColors.length
                  ][0],
                color:
                  healthStore.keywordColors[
                    index % healthStore.keywordColors.length
                  ][1],
              }}
            >
              {content}
            </span>
          );
        })}
      </div>
    </div>
  );
}
