// src/components/dashboard/StatusMetrics.jsx
import React from 'react';
import { Battery, Wifi, Thermometer, Compass, Clock, Activity } from 'lucide-react';
import DashboardCard from './DashboardCard';

const StatusMetrics = ({ metrics }) => {
  const defaultMetrics = {
    battery: "0%",
    signal: "N/A",
    temperature: "0°C",
    orientation: "0°",
    uptime: "0m",
    status: "N/A",
    ...metrics
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      <DashboardCard 
        icon={<Battery />}
        title="Status Baterai"
        value={defaultMetrics.battery}
        color="bg-green-500"
      />
      
      <DashboardCard 
        icon={<Wifi />}
        title="Kualitas Sinyal"
        value={defaultMetrics.signal}
        color="bg-blue-500"
      />

      <DashboardCard 
        icon={<Thermometer />}
        title="Suhu Motor"
        value={defaultMetrics.temperature}
        color="bg-orange-500"
      />

      <DashboardCard 
        icon={<Compass />}
        title="Orientasi"
        value={defaultMetrics.orientation}
        color="bg-purple-500"
      />

      <DashboardCard 
        icon={<Clock />}
        title="Waktu Operasi"
        value={defaultMetrics.uptime}
        color="bg-indigo-500"
      />

      <DashboardCard 
        icon={<Activity />}
        title="Status Sistem"
        value={defaultMetrics.status}
        color="bg-teal-500"
      />
    </div>
  );
};

export default StatusMetrics;