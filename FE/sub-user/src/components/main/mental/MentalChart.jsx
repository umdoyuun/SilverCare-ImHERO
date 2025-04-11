import "./Mental.css";

import { useContext } from "react";

import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  // CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";

import { CalendarStoreContext } from "../../../store/calendarStore.jsx";

function CustomTooltip({ payload, label, active }) {
  if (payload.length === 0) {
    return;
  }

  if (active) {
    return (
      <div className="custom-tooltip">
        <p className="date">{label}</p>
        <p className="mental">{`mental : ${payload[0].value}`}</p>
        <p className="desc">건강함</p>
      </div>
    );
  }

  return null;
}

export default function ActivityChart() {
  const calendarStore = useContext(CalendarStoreContext);

  let mentalStatus = Object.entries(calendarStore.schedules.schedules.mental)
    .slice()
    .map(([date, details]) => ({
      date,
      averageScore: details.averageScore,
    }));

  mentalStatus = mentalStatus.reverse();

  if (mentalStatus.length > 7) {
    mentalStatus = activityStatus.slice(0, 7);
  }

  return (
    <div id="mental-chart">
      <ResponsiveContainer
        width="100%"
        height="100%"
        Style={{ width: 100, backgroundColor: "#ccc" }}
      >
        <LineChart
          width={300}
          height={100}
          data={mentalStatus}
          margin={{
            top: 10,
            right: 50,
            bottom: 15,
          }}
        >
          <XAxis dataKey="date" />
          <YAxis />
          <Tooltip content={<CustomTooltip />} />
          <Legend />
          <Line
            type="monotone"
            dataKey="averageScore"
            stroke="#FF5A33"
            strokeWidth={2}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
