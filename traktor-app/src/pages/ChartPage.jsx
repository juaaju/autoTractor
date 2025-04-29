import React from "react";
import RealtimeChart from "/home/jetson/traktor/traktor-app/src/components/RealChart.jsx";

const API_URL = "http://ubuntu.local:5001/data_sensor_all"; // Ganti dengan IP Flask
  // Handle log file download
const handleDownloadCSV = () => {
  // Assuming your sensor fusion server is running on port 5001
  window.open('http://ubuntu.local:5001/log_file', '_blank');
};

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
      <button
        onClick={handleDownloadCSV}
        type="button"
        className="px-6 m-4 py-2 bg-blue-600 text-white font-semibold rounded-lg shadow-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-opacity-50">
        Download CSV
      </button>
    </div>
  );
}

export default ChartPage;
