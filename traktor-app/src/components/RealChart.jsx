import React, { useState, useEffect } from "react";
import Chart from "react-apexcharts";
import axios from "axios";

const RealtimeChart = ({ title, dataKey, apiUrl, yaxisRange }) => {
  const [series, setSeries] = useState([{ name: title, data: [] }]);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await axios.get(apiUrl);
        const sensorData = response.data;
        const newData = {
          x: new Date().getTime(),
          y: sensorData[dataKey], // Ambil data sesuai key yang diberikan
        };

        setSeries((prevSeries) => [
          {
            name: title,
            data: [...prevSeries[0].data.slice(-19), newData],
          },
        ]);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    const interval = setInterval(fetchData, 500); // Update tiap 0.5 detik

    return () => clearInterval(interval);
  }, [apiUrl, dataKey]);

  const options = {
    chart: {
      type: "line",
      animations: {
        enabled: true,
        easing: "linear",
        dynamicAnimation: { speed: 500 },
      },
    },
    xaxis: {
      type: "datetime",
      labels: {
        formatter: function (value) {
          const date = new Date(value);
          return date.getSeconds(); // Hanya menampilkan detik
        }
      }
    },
    yaxis: {
      min: yaxisRange?.min || -10, // Default jika tidak diberikan
      max: yaxisRange?.max || 50,  // Default jika tidak diberikan
    },
  };

  return (
    <div>
      <h3 className="m-4">{title}</h3>
      <Chart options={options} series={series} type="line" height={350} />
    </div>
  );
};

export default RealtimeChart;
