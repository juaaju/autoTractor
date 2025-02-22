// src/pages/Home.jsx
import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { GitFork, PlayCircle, LineChart, Map } from 'lucide-react';
import StatusMetrics from '../components/dashboard/StatusMetrics';
import LogActivity from '../components/dashboard/LogActivity';

const NavButton = ({ icon, text, onClick, color }) => (
  <button
    onClick={onClick}
    className={`${color} hover:bg-opacity-90 text-white font-bold py-6 px-8 rounded-lg shadow-lg flex flex-col items-center justify-center transition-all duration-300 transform hover:scale-105`}
  >
    {React.cloneElement(icon, { size: 32 })}
    <span className="mt-2 text-sm">{text}</span>
  </button>
);

export default function Home() {
  const navigate = useNavigate();
  const [metrics, setMetrics] = useState({
    battery: "85%",
    signal: "Baik",
    temperature: "45°C",
    orientation: "287°",
    uptime: "2j 15m",
    status: "Normal"
  });

  const [logs, setLogs] = useState([
    {
      time: "10:45",
      message: "Motor berputar ke kanan selama 2.5 detik",
      type: "info"
    },
    {
      time: "10:42",
      message: "Perubahan orientasi terdeteksi",
      type: "warning"
    },
    {
      time: "10:40",
      message: "Sistem dimulai dalam mode manual",
      type: "success"
    }
  ]);

  useEffect(() => {
    const interval = setInterval(() => {
      // Update metrics secara periodik
      // fetchMetrics();
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  return (
    <div className="h-[calc(100vh-2rem)] overflow-y-auto">
      <div className="mb-8">
        <h1 className="flex font-bold m-4 text-3xl text-gray-800 justify-center">Selamat Datang</h1>
      </div>

      {/* Navigation Buttons */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-8">
        <NavButton
          icon={<GitFork />}
          text="Kontrol Manual"
          onClick={() => navigate('/motor-control')}
          color="bg-blue-500"
        />
        <NavButton
          icon={<PlayCircle />}
          text="Kontrol Otomatis"
          onClick={() => navigate('/auto')}
          color="bg-green-500"
        />
        <NavButton
          icon={<LineChart />}
          text="Grafik IMU-GPS"
          onClick={() => navigate('/chart')}
          color="bg-purple-500"
        />
        <NavButton
          icon={<Map />}
          text="Peta Realtime"
          onClick={() => navigate('/map')}
          color="bg-orange-500"
        />
      </div>

      <StatusMetrics metrics={metrics} />
      
      <div className="mt-8">
        <LogActivity logs={logs} />
      </div>
    </div>
  );
}