import { useState, useContext, createContext } from "react";
import { useHttp } from "../hooks/useHttp";

import { UserProgressContext } from "./userProgressStore";

export const HealthContext = createContext({
  healthStatus: [
    {
      index: 1,
      family_id: "",
      reported_at: "",
      heart_rate: 1,
    },
  ],
  activityStatus: [
    {
      index: 1,
      family_id: "",
      reported_at: "",
      score: 0,
      action: "",
      is_critical: false,
      description: { title: "", data: 123.456 },
    },
  ],
  mentalStatus: [
    {
      index: 1,
      family_id: "",
      reported_at: "",
      score: 5,
      is_critical: false,
      description: {
        overall_emotional_state: "",
        emotional_insights: "",
        time_based_emotions: {},
        recommendations: [],
      },
    },
  ],
  mentalReport: [
    {
      index: 3,
      family_id: "FlcuDLxVC9SolW70",
      reported_at: "2025-02-07T00:31:04",
      start_time: "2025-02-01T00:30:24",
      end_time: "2025-02-06T23:59:59",
      average_score: 90,
      critical_days: 2,
      best_day: "2025-02-03",
      worst_day: "2025-02-05",
      improvement_needed: false,
      summary: "또 다른 총평 요약",
    },
  ],
  mentalHealthStatus: {
    language_patterns: {
      score: 60,
      word_choice:
        "사용자는 '안녕', '뭐 해', '그냥 불러 봤어' 등의 단어를 반복적으로 사용하고 있습니다. 이는 사용자가 특정 단어에 고착화되어 있거나, 그 단어를 통해 특정 감정을 표현하려는 의도가 있을 수 있습니다.",
      sentence_structure:
        "사용자의 문장 구조는 대체로 일관되어 있지만, 일부 문장에서는 주어나 동사가 빠진 경우가 있습니다. 이는 사용자가 문장을 완전히 표현하는데 어려움을 겪고 있을 수 있습니다.",
      expression_clarity:
        "사용자의 표현은 대체로 명확하지만, 일부 문장에서는 맥락이 불분명하거나 불완전한 문장을 사용하는 경우가 있습니다.",
    },
    contextual_analysis: {
      score: 50,
      topic_flow:
        "사용자는 대화 주제를 자연스럽게 전환하는데 어려움이 있습니다. 대화 주제가 불분명하거나 반복적으로 같은 주제로 돌아오는 경향이 있습니다.",
      context_understanding:
        "사용자는 대화 맥락을 이해하는데 어려움이 있을 수 있습니다. 동일한 질문을 반복하거나, 이전 대화 내용과 관련 없는 주제로 전환하는 경우가 있습니다.",
    },
    cognitive_state: {
      score: 55,
      clarity:
        "사용자의 사고는 대체로 명확하지만, 일부 문장에서는 불분명하거나 혼란스러울 수 있습니다.",
      reality_perception:
        "사용자는 현실 인식에 어려움이 있을 수 있습니다. 일부 문장에서는 현실과 다른 상황을 이야기하는 것으로 보입니다.",
    },
    overall_assessment: {
      total_score: 55,
      risk_level: "medium",
      concerns: ["언어 패턴의 반복", "대화 맥락 이해도 부족", "현실 인식 문제"],
      strengths: ["일관된 문장 구조", "대체로 명확한 표현"],
    },
    recommendations: [
      "전문가와 상담을 통해 정신 건강 상태를 평가받아보는 것이 좋습니다.",
      "일상생활에서 스트레스를 줄이는 방법을 찾아보세요.",
      "정기적인 운동과 영양 균형 잡힌 식사를 통해 건강을 유지하세요.",
    ],
    analysis_period: {
      start: "2025-02-06T15:00:00",
      end: "2025-02-13T14:59:59",
    },
    data_stats: {
      total_conversations: 59,
      analysis_type: "periodic",
    },
  },
  selectedToggle: "",
  weeklyData: [{ name: "", value: 0 }],
  keywords: [],
  keywordColors: [],
  healthLog: {
    data: [
      {
        date: "",
        health: 1,
        mental: 1,
      },
    ],
    mentalReport: {
      data: [
        {
          id: 3,
          user_id: "",
          created_at: "",
          report_content: {
            overall_emotional_state: "",
            emotional_insights: "",
            time_based_emotions: {},
            recommendations: [],
          },
        },
      ],
    },
  },
  handleChangeSelectedToggle: (toggle) => {},
  handleShowDetailReport: () => {},
  handleGetHealthData: () => {},
  handleGetActivityStatus: () => {},
  handleGetMentalStatus: () => {},
  handleGetMentalReports: () => {},
  handleGetOneDayMentalReport: () => {},
  handleGetMentalHealthDailyStatus: () => {},
  handleGetMentalHealthPeriodStatus: (inputStart, inputEnd) => {},
  handleShowMentalHealthReport: () => {},
  handleCloseMentalHealthReport: () => {},
  handleGetKeywords: () => {},
});

export default function HealthContextProvider({ children }) {
  const userProgressStore = useContext(UserProgressContext);

  const { request, loading } = useHttp();

  const [healthStatus, setHealthStatus] = useState([]);
  const [activityStatus, setActivityStatus] = useState([]);
  const [mentalStatus, setMentalStatus] = useState([]);
  const [mentalHealthStatus, setMentalHealthStatus] = useState({});

  const [selectedToggle, setSelectedToggle] = useState("activity");
  const [mentalReport, setMentalReport] = useState([]);

  const [weeklyData, setWeeklyData] = useState([
    { name: "health", value: 0 },
    { name: "mental", value: 0 },
  ]);

  const [keywords, setKeywords] = useState([]);

  // 대화 키워드 관련련
  // const keywords = ["임영웅", "김치찌개", "두부", "여행", "병원"];
  const keywordColors = [
    ["#146152", "white"],
    ["#44803F", "white"],
    ["#B4CF66", "black"],
    ["#FFEC5C", "black"],
    ["#FF5A33", "white"],
  ];

  // 건강 점수수
  const healthLog = [];

  let familyId = "";
  if (userProgressStore.loginUserInfo.userInfo?.role === "sub") {
    familyId = userProgressStore.memberInfo.selectedFamilyId;
  } else if (userProgressStore.loginUserInfo.userInfo?.role === "main") {
    familyId = userProgressStore.familyInfo?.familyInfo?.id;
  }

  function handleChangeSelectedToggle(toggle) {
    setSelectedToggle(toggle);
  }

  // 상세 보고서 모달 열기
  function handleShowDetailReport(id = 0) {
    userProgressStore.handleOpenModal("detail-mental-report", id);
  }

  // 1년 전부터 현재 시간까지의 범위를 UTC 기준으로 출력하는 함수
  function getOneYearRangeUTC() {
    const now = new Date(); // 현재 시간 (UTC)
    const oneYearAgo = new Date();
    oneYearAgo.setUTCFullYear(now.getUTCFullYear() - 1); // UTC 기준 1년 전

    const formatUTCDate = (date) => date.toISOString().split(".")[0] + "Z"; // 밀리초 제거 후 'Z' 추가

    return {
      start: formatUTCDate(oneYearAgo),
      end: formatUTCDate(now),
    };
  }

  // 7일 전부터 현재 시간까지의 범위를 UTC 기준으로 출력하는 함수
  const getSevenDaysRangeUTC = () => {
    const now = new Date();

    // 7일 전 날짜 계산
    const sevenDaysAgo = new Date();
    sevenDaysAgo.setUTCDate(now.getUTCDate() - 7);

    // 날짜를 ISO 8601 형식으로 변환하고, 밀리초 제거 후 'Z' 추가
    const formatUTCDate = (date) => date.toISOString().split(".")[0] + "Z";

    return {
      start: formatUTCDate(sevenDaysAgo),
      end: formatUTCDate(now),
    };
  };

  async function handleGetHealthData(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await fetch(
        `${userProgressStore.DEV_API_URL}/status/health/${familyId}?start=${start}&end=${end}&order=${order}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include",
        }
      );

      const resData = await response.json().catch(() => null);

      if (response.ok) {
        if (resData.message === "Health status retrieved successfully") {
          setHealthStatus([...resData.result]);
          return {
            success: true,
            data: resData.result,
          };
        } else if (resData.message === "No health status found") {
          setHealthStatus([...resData.result]);
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error("최신 건강 정보 조회 실패:", resData.error);
        setHealthStatus([]);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      setHealthStatus([]);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetActivityStatus(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/active/${familyId}?start=${inputStart}&end=${inputEnd}&order=${order}`
      );

      // console.log("handleGetActivityStatus response", response);
      const resData = response.data;

      if (response.success) {
        if (resData.message === "Active status retrieved successfully") {
          setActivityStatus(
            resData.result.map((item) => ({
              ...item,
              description: JSON.parse(item.description.replace(/'/g, '"')),
            }))
          );
          return {
            success: true,
            data: resData.result,
          };
        } else if (resData.message === "No active status found") {
          setActivityStatus([...resData.result]);
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error("최신 활동 정보 조회 실패:", resData.error);
        setActivityStatus([]);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      setActivityStatus([]);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetMentalStatus(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/mental/${familyId}?start=${inputStart}&end=${inputEnd}&order=${order}`
      );

      // console.log("handleGetMentalStatus response", response);
      const resData = response.data;

      if (response.success) {
        if (resData.message === "Mental status retrieved successfully") {
          setMentalStatus(
            resData.result.map((item) => ({
              ...item,
              description: JSON.parse(item.description.replace(/'/g, '"')),
            }))
          );
          return {
            success: true,
            data: resData.result,
          };
        } else if (resData.message === "No mental status found") {
          setMentalStatus([...resData.result]);
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error("최신 정신 정보 조회 실패:", resData.error);
        setMentalStatus([]);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      setMentalStatus([]);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetMentalReports(
    inputStart = null,
    inputEnd = null,
    order = "desc"
  ) {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    const { start, end } = getOneYearRangeUTC();

    if (!inputStart) {
      inputStart = start;
    }

    if (!inputEnd) {
      inputEnd = end;
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/mental-reports/${familyId}?start=${inputStart}&end=${inputEnd}&order=${order}`
      );

      // console.log("handleGetMentalReports response", response);
      const resData = response.data;

      if (response.success) {
        if (resData.message === "Mental reports retrieved successfully") {
          setMentalReport([...resData.result]);
          return {
            success: true,
            data: resData.result,
          };
        } else if (resData.message === "No mental reports found") {
          setMentalReport([...resData.result]);
          return {
            success: true,
            data: resData.result,
          };
        }
      } else {
        console.error("최신 정신 보고서 조회 실패:", resData.error);
        setMentalReport([]);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error(error);
      setMentalReport([]);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetOneDayMentalReport() {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return;
    }

    try {
      // const response = await request(`${userProgressStore.DEV_API_URL}/status/mental/new/${familyId}`, 'POST', {})
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/mental/new/${familyId}`,
        "GET"
      );

      const resData = response.data;

      if (response.success) {
        if (true) {
          // console.log("handleGetOneDayMentalReport response", resData);
          await handleGetMentalReports();
          await handleGetMentalStatus();
        }
      }
    } catch (error) {
      console.error("최신 정신 건강 보고서를 요청하는 데 실패했습니다.", error);
    }
  }

  const handleGetWeekData = async () => {
    if (!userProgressStore.memberInfo.selectedFamilyId) return;

    const { start, end } = getSevenDaysRangeUTC();

    try {
      // 첫 번째 API 호출: 활동 상태
      const response = await handleGetActivityStatus(start, end, "desc");

      if (response.success && response.data.length > 0) {
        const health =
          response.data.reduce((acc, entry) => acc + entry.score, 0) /
          response.data.length;
        // health 값 업데이트
        setWeeklyData((prev) =>
          prev.map((item) =>
            item.name === "health" ? { ...item, value: health } : item
          )
        );
      } else {
        // 데이터가 없을 경우 초기화
        setWeeklyData((prev) =>
          prev.map((item) =>
            item.name === "health" ? { ...item, value: 0 } : item
          )
        );
      }

      // 두 번째 API 호출: 정신 상태
      const secResponse = await handleGetMentalStatus(start, end, "desc");

      if (secResponse.success && secResponse.data.length > 0) {
        const mental =
          secResponse.data.reduce((acc, entry) => acc + entry.score, 0) /
          secResponse.data.length;

        // mental 값 업데이트
        setWeeklyData((prev) =>
          prev.map((item) =>
            item.name === "mental" ? { ...item, value: mental } : item
          )
        );
      } else {
        // 데이터가 없을 경우 초기화
        setWeeklyData((prev) =>
          prev.map((item) =>
            item.name === "mental" ? { ...item, value: 0 } : item
          )
        );
      }
    } catch (error) {
      console.error("활동 상태 데이터를 불러오는 데 실패했습니다.", error);
    }
  };

  async function handleGetMentalHealthDailyStatus() {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/psychology/${familyId}`
      );

      // console.log("handleGetMentalHealthDailyStatus response", response);
      const resData = response.data;

      if (response.success) {
        if (resData.message === "Psychology report created successfully") {
          // console.log("정신 건강 상태 불러오기 성공:", resData.result);
          setMentalHealthStatus({ ...resData.result });
          return {
            success: true,
            data: resData.result,
          };
        } else {
          console.error("정신 건강 상태 불러오기 실패:", resData.error);
          setMentalHealthStatus({});
          return {
            success: false,
            error: {
              type: resData.error.type,
              message: resData.error.message,
            },
          };
        }
      }
    } catch (error) {
      console.error("정신 건강 상태를 불러오는 데 실패했습니다.", error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  async function handleGetMentalHealthPeriodStatus(inputStart, inputEnd) {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/psychology/${familyId}?start=${inputStart}&end=${inputEnd}`
      );

      const resData = response.data;

      if (response.success) {
        if (resData.message === "Psychology report created successfully") {
          // console.log("정신 건강 상태 불러오기 성공:", resData.result);
          setMentalHealthStatus({ ...resData.result });
          return {
            success: true,
            data: resData.result,
          };
        } else {
          console.error("정신 건강 상태 불러오기 실패:", resData.error);
          setMentalHealthStatus({});
          return {
            success: false,
            error: {
              type: resData.error.type,
              message: resData.error.message,
            },
          };
        }
      }
    } catch (error) {
      console.error("정신 건강 상태를 불러오는 데 실패했습니다.", error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  function handleShowMentalHealthReport() {
    userProgressStore.handleOpenModal("mental-health-report");
  }

  function handleCloseMentalHealthReport() {
    userProgressStore.handleCloseModal();
    setMentalHealthStatus({});
  }

  async function handleGetKeywords() {
    if (!familyId) {
      console.error("가족 ID가 없습니다.");
      return {
        success: false,
        error: {
          type: "no_family_id",
          message: "가족 ID가 없습니다.",
        },
      };
    }

    try {
      const response = await request(
        `${userProgressStore.DEV_API_URL}/status/keywords/${familyId}`
      );

      const resData = response.data;

      if (response.success) {
        if (resData.message === "Conversation keywords created successfully") {
          // console.log("대화 키워드 불러오기 성공:", resData.result);
          setKeywords([...resData.result.keywords]);
        }
      } else {
        console.error("대화 키워드 불러오기 실패:", resData.error);
        return {
          success: false,
          error: {
            type: resData.error.type,
            message: resData.error.message,
          },
        };
      }
    } catch (error) {
      console.error("대화 키워드를 불러오는 데 실패했습니다.", error);
      return {
        success: false,
        error: {
          type: "network_error",
          message: "네트워크 오류가 발생했습니다.",
        },
      };
    }
  }

  const ctxValue = {
    loading,
    healthStatus,
    activityStatus,
    mentalStatus,
    mentalReport,
    mentalHealthStatus,
    selectedToggle,
    weeklyData,
    keywords,
    keywordColors,
    healthLog,
    handleChangeSelectedToggle,
    handleShowDetailReport,
    handleGetHealthData,
    handleGetActivityStatus,
    handleGetMentalStatus,
    handleGetMentalReports,
    handleGetOneDayMentalReport,
    handleGetWeekData,
    handleGetMentalHealthDailyStatus,
    handleGetMentalHealthPeriodStatus,
    handleShowMentalHealthReport,
    handleCloseMentalHealthReport,
    handleGetKeywords,
  };

  return (
    <HealthContext.Provider value={ctxValue}>{children}</HealthContext.Provider>
  );
}
