import React from "react";
import RealtimeChart from "/home/jetson/traktor/traktor-app/src/components/RealChart.jsx";

const API_URL = "http://ubuntu.local:5001/data_sensor"; // Ganti dengan IP Flask

function ChartPage() {
  return (
    <div>
      <h1 className="flex font-bold m-4 text-3xl text-gray-800 justify-center">Real-time Sensor Data</h1>
      <RealtimeChart title="Hasil EKF_X" dataKey="ekf_x" apiUrl={API_URL} />
      <RealtimeChart title="Hasil EKF_Y" dataKey="ekf_y" apiUrl={API_URL} />
      <RealtimeChart title="Data MPU_X" dataKey="imu_x" apiUrl={API_URL} />
      <RealtimeChart title="Data MPU_Y" dataKey="imu_y" apiUrl={API_URL} />
      <RealtimeChart title="Data GPS_LAT" dataKey="ekf_lat" apiUrl={API_URL} />
      <RealtimeChart title="Data GPS_LON" dataKey="ekf_lng" apiUrl={API_URL} />
    </div>
  );
}

export default ChartPage;
