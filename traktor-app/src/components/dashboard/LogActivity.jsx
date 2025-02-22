// src/components/dashboard/LogActivity.jsx
import React from 'react';

const LogItem = ({ time, message, type }) => {
  const colors = {
    info: "text-blue-600",
    warning: "text-orange-600",
    success: "text-green-600"
  };

  return (
    <div className="flex items-center space-x-3">
      <span className="text-sm text-gray-500">{time}</span>
      <span className={`text-sm ${colors[type]}`}>{message}</span>
    </div>
  );
};

const LogActivity = ({ logs = [] }) => {
  return (
    <div className="bg-white p-6 rounded-lg shadow-md">
      <h2 className="text-xl font-semibold text-gray-800 mb-4">Log Aktivitas Terakhir</h2>
      <div className="space-y-3">
        {logs.map((log, index) => (
          <LogItem 
            key={index}
            time={log.time}
            message={log.message}
            type={log.type}
          />
        ))}
      </div>
    </div>
  );
};

export default LogActivity;