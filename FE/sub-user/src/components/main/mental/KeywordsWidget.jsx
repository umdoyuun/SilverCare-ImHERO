import "./Mental.css"

import { useContext } from "react"

import { HealthContext } from "../../../store/healthStore"

import Widget from "../../widget/Widget"

export default function KeywordsWidget() {
  const healthStore = useContext(HealthContext)

  return (
    <Widget title={`일간 대화 키워드`} type="keyword">
      <div id="keywords-body">
        {healthStore.keywords.length === 0 && (
          <span
            id="keyword-box"
            style={{
              backgroundColor: healthStore.keywordColors[0][0],
              color: healthStore.keywordColors[0][1],
            }}
          >
            분석된 대화 내용이 없습니다.
          </span>
        )}
        {healthStore.keywords.length > 0 &&
          healthStore.keywords.map((content, index) => {
            return (
              <span
                key={index}
                id="keyword-box"
                style={{
                  backgroundColor: healthStore.keywordColors[index % healthStore.keywordColors.length][0],
                  color: healthStore.keywordColors[index % healthStore.keywordColors.length][1],
                }}
              >
                {content}
              </span>
            )
          })}
      </div>
    </Widget>
  )
}
