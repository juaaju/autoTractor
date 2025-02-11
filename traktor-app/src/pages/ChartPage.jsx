import React from "react";
import RealtimeChart from "/home/jetson/traktor/traktor-app/src/components/RealChart.jsx";

const API_URL = "http://ubuntu.local:5000/get_data"; // Ganti dengan IP Flask

function ChartPage() {
  return (
    <div>
      <h1 className="flex font-bold m-4 text-3xl text-gray-800 justify-center">Real-time Sensor Data</h1>
      <RealtimeChart title="Hasil Fusi Kalman Filter" dataKey="temperature" apiUrl={API_URL} yaxisRange={{ min: 0, max: 50 }}/>
      <RealtimeChart title="Data MPU-6050" dataKey="accel.x" apiUrl={API_URL} yaxisRange={{ min: -2, max: 2}}/>
      <RealtimeChart title="Data GPS" dataKey="accel.x" apiUrl={API_URL} yaxisRange={{ min: -2, max: 2}}/>
    </div>
  );
}

export default ChartPage;
