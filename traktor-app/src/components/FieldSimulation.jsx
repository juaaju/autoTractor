import React, { useState, useEffect } from 'react';

const FieldSimulation = () => {
  const [tractorPosition, setTractorPosition] = useState({ x: 50, y: 50 });
  const [pathData, setPathData] = useState({
    fieldPoints: [],
    gpsRaw: [],
    imuRaw: [],
    ekf: []
  });

  // Generate random path points
  const generateRandomPath = (baseX, baseY, noise = 0) => {
    const points = [];
    for (let i = 0; i < 100; i++) {
      points.push({
        x: baseX + i * 3 + (Math.random() - 0.5) * noise,
        y: baseY + Math.sin(i * 0.1) * 20 + (Math.random() - 0.5) * noise
      });
    }
    return points;
  };

  // Initialize simulation data
  useEffect(() => {
    // Generate corner points for field boundary
    const fieldCorners = [
      { x: 50, y: 50 },
      { x: 350, y: 50 },
      { x: 350, y: 250 },
      { x: 50, y: 250 }
    ];

    setPathData({
      fieldPoints: fieldCorners,
      gpsRaw: generateRandomPath(50, 150, 10),    // More noisy
      imuRaw: generateRandomPath(50, 150, 5),     // Less noisy
      ekf: generateRandomPath(50, 150, 2)         // Smoothest
    });
  }, []);

  // Animate tractor movement
  useEffect(() => {
    let currentIndex = 0;
    const interval = setInterval(() => {
      if (pathData.ekf[currentIndex]) {
        setTractorPosition(pathData.ekf[currentIndex]);
        currentIndex = (currentIndex + 1) % pathData.ekf.length;
      }
    }, 50);

    return () => clearInterval(interval);
  }, [pathData.ekf]);

  return (
    <div className="bg-white p-6 rounded-lg shadow-lg">
      <h2 className="text-xl font-bold mb-4">Pergerakan Traktor</h2>
      
      <div className="relative">
        <svg width="400" height="300" className="border border-gray-200">
          {/* Field boundary */}
          <path
            d={`M ${pathData.fieldPoints.map(p => `${p.x},${p.y}`).join(' L ')} Z`}
            fill="none"
            stroke="black"
            strokeWidth="2"
          />

          {/* GPS Raw Path */}
          <path
            d={pathData.gpsRaw.length > 0 ? 
              `M ${pathData.gpsRaw.map(p => `${p.x},${p.y}`).join(' L ')}` : ''}
            fill="none"
            stroke="red"
            strokeWidth="1"
            strokeDasharray="4"
          />

          {/* IMU Raw Path */}
          <path
            d={pathData.imuRaw.length > 0 ? 
              `M ${pathData.imuRaw.map(p => `${p.x},${p.y}`).join(' L ')}` : ''}
            fill="none"
            stroke="blue"
            strokeWidth="1"
            strokeDasharray="4"
          />

          {/* EKF Path */}
          <path
            d={pathData.ekf.length > 0 ? 
              `M ${pathData.ekf.map(p => `${p.x},${p.y}`).join(' L ')}` : ''}
            fill="none"
            stroke="green"
            strokeWidth="2"
          />

          {/* Tractor */}
          <g transform={`translate(${tractorPosition.x},${tractorPosition.y})`}>
            <rect
              x="-10"
              y="-5"
              width="20"
              height="10"
              fill="brown"
            />
            <circle
              cx="8"
              cy="0"
              r="2"
              fill="black"
            />
          </g>
        </svg>

        {/* Legend */}
        <div className="absolute top-2 right-2 bg-white bg-opacity-80 p-2 rounded">
          <div className="flex items-center mb-1">
            <div className="w-4 h-0.5 bg-red-500 mr-2" />
            <span className="text-sm">GPS Raw</span>
          </div>
          <div className="flex items-center mb-1">
            <div className="w-4 h-0.5 bg-blue-500 mr-2" />
            <span className="text-sm">IMU Raw</span>
          </div>
          <div className="flex items-center">
            <div className="w-4 h-0.5 bg-green-500 mr-2" />
            <span className="text-sm">EKF</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default FieldSimulation;