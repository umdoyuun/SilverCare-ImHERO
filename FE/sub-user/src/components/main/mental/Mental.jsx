import "./Mental.css";

import PageContainer from "../container/PageContainer";

import Keywords from "./Keywords.jsx";
import MentalChartContainer from "./MentalChartContainer.jsx";
import MentalReport from "./MentalReport.jsx";
import MentalHealth from "./MentalHealth.jsx";

export default function Mental() {
  const mainUserName = "박순자123";

  return (
    <div id="mental-main">
      <div id="mental-container">
        <div id="mental-elem-left">
          <PageContainer title={`${mainUserName}님의 정신 건강 상태`}>
            <MentalChartContainer />
          </PageContainer>
          <Keywords />
        </div>
        <div id="mental-elem-right">
          <PageContainer title="대화 리포트">
            <MentalReport />
          </PageContainer>
          <PageContainer title="정신 건강">
            <MentalHealth />
          </PageContainer>
        </div>
      </div>
    </div>
  );
}
