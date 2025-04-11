import "./Activity.css";

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
        <p className="health">{`health : ${payload[0].value}`}</p>
        <p className="desc">건강함</p>
      </div>
    );
  }

  return null;
}

export default function ActivityChart() {
  const calendarStore = useContext(CalendarStoreContext);

  let activityStatus = Object.entries(calendarStore.schedules.schedules.health)
    .slice()
    .map(([date, details]) => ({
      date,
      averageScore: details.averageScore,
    }));

  activityStatus = activityStatus.reverse();

  if (activityStatus.length > 7) {
    activityStatus = activityStatus.slice(0, 7);
  }

  return (
    <div id="activity-chart">
      <ResponsiveContainer
        width="100%"
        height="100%"
        Style={{ width: 100, backgroundColor: "#ccc" }}
      >
        <LineChart
          width={300}
          height={100}
          data={activityStatus}
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
            stroke="#44803F"
            strokeWidth={2}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
