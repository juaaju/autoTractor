import React from "react";
import RealtimeChart from "/home/jetson/traktor/web-app/traktor-app/src/components/RealChart.jsx";

function ChartPage() {
  return (
    <div>
      <h1 className="flex justify-center m-3">Data MPU-6050</h1>
      <RealtimeChart />
      <h1 className="flex justify-center m-3">Data GPS</h1>
      <RealtimeChart />
      <h1 className="flex justify-center m-3">Hasil Filter</h1>
      <RealtimeChart />
    </div>
  );
}

export default ChartPage;
