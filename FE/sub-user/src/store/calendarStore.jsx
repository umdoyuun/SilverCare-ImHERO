import { useState, createContext, useEffect } from "react";
import useCalendar from "../hooks/useCalendar";
import { set } from "date-fns";

export const CalendarStoreContext = createContext({
  currentDate: {
    year: "",
    month: "",
    day: "",
  },
  daysInMonth: {
    date: "",
    year: "",
    month: "",
    day: "",
    dayIndexOfWeek: 1,
  },
  dispatch: {
    handlePrevYear: () => {},
    handleNextYear: () => {},
    handlePrevMonth: () => {},
    handleNextMonth: () => {},
  },
  selectedDate: {
    date: "",
    selectedDate: () => {},
  },
  groupDataByKST: (data) => {},
  groupHomeStatusDataByKSTWithAvgScore: (data) => {},
  groupDataByKSTWithAvgScore: (identifier, data) => {},
  schedules: {
    schedules: {
      mental: {},
      health: {},
      homeStatus: {},
    },
    addSchedule: () => {},
  },
});

export default function CalendarStoreContextProvider({ children }) {
  const context = useCalendar();

  const [schedules, setSchedules] = useState({
    mental: {},
    health: {},
    homeStatus: {},
  });

  function groupDataByKST(data) {
    const groupedData = {};

    data.forEach((item) => {
      // UTC -> KST 변환 (UTC+9)
      const kstDate = new Date(item.reported_at);
      kstDate.setHours(kstDate.getHours() + 9);

      // YYYY-MM-DD 형식으로 변환
      const dateKey = kstDate.toISOString().split("T")[0];

      // 날짜별 데이터 저장
      if (!groupedData[dateKey]) {
        groupedData[dateKey] = [];
      }
      groupedData[dateKey].push({
        ...item,
        reported_at: kstDate.toISOString(), // KST 변환된 시간 저장
      });
    });

    return groupedData;
  }

  function groupHomeStatusDataByKSTWithAvgScore(data) {
    // 데이터가 없을 경우 초기화
    if (Object.keys(data).length === 0) {
      setSchedules((prevSchedules) => {
        return {
          ...prevSchedules,
          homeStatus: {},
        };
      });

      return {};
    }

    const groupedData = {};

    data.forEach((entry) => {
      // 1. UTC 시간을 KST로 변환
      const kstDate = new Date(entry.reported_at);
      kstDate.setHours(kstDate.getHours() + 9); // UTC → KST 변환
      const dateKey = kstDate.toISOString().split("T")[0]; // YYYY-MM-DD 형식

      // 2. 그룹화된 데이터 생성
      if (!groupedData[dateKey]) {
        groupedData[dateKey] = {
          count: 0,
          temperature: 0,
          humidity: 0,
          dust_level: 0,
          ethanol: 0,
        };
      }

      groupedData[dateKey].temperature += entry.temperature;
      groupedData[dateKey].humidity += entry.humidity;
      groupedData[dateKey].dust_level += entry.dust_level;
      groupedData[dateKey].ethanol += entry.ethanol;
      groupedData[dateKey].count += 1;
    });

    // 3. 평균값 계산
    Object.keys(groupedData).forEach((date) => {
      const data = groupedData[date];
      groupedData[date] = {
        temperature: data.temperature / data.count,
        humidity: data.humidity / data.count,
        dust_level: data.dust_level / data.count,
        ethanol: data.ethanol / data.count,
      };
    });

    setSchedules((prevSchedules) => {
      return {
        ...prevSchedules,
        homeStatus: groupedData,
      };
    });

    return groupedData;
  }

  function groupDataByKSTWithAvgScore(identifier, data) {
    const groupedData = {};

    if (!data || Object.keys(data).length === 0) {
      setSchedules((prevSchedules) => {
        return {
          ...prevSchedules,
          [identifier]: {},
        };
      });

      return {};
    }

    data.forEach((item) => {
      // UTC -> KST 변환 (UTC+9)
      const kstDate = new Date(item.reported_at);
      kstDate.setHours(kstDate.getHours() + 9);

      // YYYY-MM-DD 형식으로 변환
      const dateKey = kstDate.toISOString().split("T")[0];

      // 날짜별 데이터 저장 및 점수 합산
      if (!groupedData[dateKey]) {
        groupedData[dateKey] = { records: [], totalScore: 0, count: 0 };
      }

      groupedData[dateKey].records.push({
        ...item,
        reported_at: kstDate.toISOString(), // KST 변환된 시간 저장
      });

      // 점수 합산 및 개수 증가
      groupedData[dateKey].totalScore += item.score;
      groupedData[dateKey].count += 1;
    });

    // 평균 점수 계산
    const result = {};
    Object.keys(groupedData).forEach((date) => {
      const { records, totalScore, count } = groupedData[date];
      result[date] = {
        records,
        averageScore: count > 0 ? totalScore / count : 0, // 평균 점수 계산
      };
    });

    setSchedules((prevSchedules) => {
      return {
        ...prevSchedules,
        [identifier]: result,
      };
    });

    return result;
  }

  // 상태 관리

  // 상태 변경 시 localStorage에 저장
  // useEffect(() => {
  //   localStorage.setItem("schedules", JSON.stringify(schedules))
  // }, [schedules])

  // 스케줄 추가 함수
  const addSchedule = (date, schedule) => {
    setSchedules((prevSchedules) => {
      const updatedSchedules = {
        ...prevSchedules,
        [date]: prevSchedules[date]
          ? [...prevSchedules[date], schedule]
          : [schedule],
      };
      return updatedSchedules;
    });
  };

  const ctxValue = {
    ...context,
    groupDataByKST,
    groupHomeStatusDataByKSTWithAvgScore,
    groupDataByKSTWithAvgScore,
    schedules: {
      schedules,
      addSchedule,
    },
  };

  // console.log(ctxValue)
  return (
    <CalendarStoreContext.Provider value={ctxValue}>
      {children}
    </CalendarStoreContext.Provider>
  );
}
