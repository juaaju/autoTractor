import React, { useState, useEffect } from "react";
import Chart from "react-apexcharts";

const RealtimeChart = () => {
  const [series, setSeries] = useState([
    { name: "Sensor Data", data: [] }, // Data awal kosong
  ]);

  useEffect(() => {
    const interval = setInterval(() => {
      setSeries((prevSeries) => {
        const newData = {
          x: new Date().getTime(), // Waktu sekarang
          y: Math.floor(Math.random() * 100), // Data random, ganti dengan data dari API atau WebSocket
        };

        return [
          {
            ...prevSeries[0],
            data: [...prevSeries[0].data.slice(-19), newData], // Simpan max 20 data
          },
        ];
      });
    }, 500); // Update tiap 1 detik

    return () => clearInterval(interval);
  }, []);

  const options = {
    chart: {
      type: "line",
      animations: {
        enabled: true,
        easing: "linear",
        dynamicAnimation: { speed: 1000 }, // Animasi data masuk
      },
    },
    xaxis: { type: "datetime" },
    yaxis: { min: 0, max: 100 },
  };

  return <Chart options={options} series={series} type="line" height={350} />;
};

export default RealtimeChart;
