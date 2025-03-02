import React, { useState, useEffect, useRef } from "react";  // Added useRef here
import Chart from "react-apexcharts";
import axios from "axios";

const RealtimeChart = ({ title, dataKey, apiUrl, yaxisRange }) => {
  const [series, setSeries] = useState([{ name: title, data: [] }]);
  const [dynamicYAxis, setDynamicYAxis] = useState({ min: 0, max: 10 });
  const isMounted = useRef(true);
  const intervalRef = useRef(null);

  useEffect(() => {
    // Set up component
    isMounted.current = true;
    
    const fetchData = async () => {
      if (!isMounted.current) return;
      
      try {
        const response = await axios.get(apiUrl, { timeout: 2000 });
        if (!isMounted.current) return;
        
        const sensorData = response.data;
        const newValue = sensorData[dataKey];
        
        const newData = {
          x: sensorData.timestamp,
          y: newValue,
        };

        // Only update state if component is still mounted
        if (isMounted.current) {
          setSeries((prevSeries) => {
            const updatedData = [...prevSeries[0].data.slice(-19), newData];
            
            // Calculate dynamic y-axis range based on current data
            if (updatedData.length > 0) {
              const values = updatedData.map(item => item.y).filter(val => val !== null && val !== undefined);
              
              if (values.length > 0) {
                const minValue = Math.min(...values);
                const maxValue = Math.max(...values);
                
                // Add padding (20% of the range)
                const padding = (maxValue - minValue) * 0.2 || 5;
                
                setDynamicYAxis({
                  min: Math.floor(minValue - padding),
                  max: Math.ceil(maxValue + padding)
                });
              }
            }
            
            return [
              {
                name: title,
                data: updatedData,
              },
            ];
          });
        }
      } catch (error) {
        console.error("Error fetching data:", error);
        // Don't keep trying if there's a persistent error
        if (error.response && error.response.status >= 400) {
          clearInterval(intervalRef.current);
        }
      }
    };

    // Create interval and store reference
    intervalRef.current = setInterval(fetchData, 500);
    
    // Clean up function
    return () => {
      isMounted.current = false;
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [apiUrl, dataKey, title]);

  // Rest of your chart code remains the same
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
      // Use either manually provided yaxisRange or calculated dynamicYAxis
      min: yaxisRange?.min !== undefined ? yaxisRange.min : dynamicYAxis.min,
      max: yaxisRange?.max !== undefined ? yaxisRange.max : dynamicYAxis.max,
      floating: false,
      decimalsInFloat: 1,
      labels: {
        formatter: (value) => value.toFixed(1)
      }
    },
    tooltip: {
      x: {
        format: 'HH:mm:ss'
      }
    }
  };

  return (
    <div>
      <h3 className="m-4">{title}</h3>
      <Chart options={options} series={series} type="line" height={350} />
    </div>
  );
};

export default RealtimeChart;