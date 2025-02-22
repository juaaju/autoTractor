// src/pages/SimulationPage.jsx
import React, { useState } from 'react';
import FieldSimulation from '../components/FieldSimulation';

const SimulationPage = () => {
  const [showGPS, setShowGPS] = useState(true);
  const [showIMU, setShowIMU] = useState(true);
  const [showEKF, setShowEKF] = useState(true);

  return (
    <div className="mb-6">
      <div className="mb-6">
      </div>

      <div className="grid md:grid-cols-2 gap-6">
        <div>
          <FieldSimulation 
            showGPS={showGPS}
            showIMU={showIMU}
            showEKF={showEKF}
          />
        </div>

        <div className="bg-white p-6 rounded-lg shadow-lg">
          <h2 className="text-xl font-bold mb-4">Pengaturan Visualisasi</h2>
          
          <div className="space-y-4">
            <div className="flex items-center">
              <input
                type="checkbox"
                id="showGPS"
                checked={showGPS}
                onChange={(e) => setShowGPS(e.target.checked)}
                className="mr-2"
              />
              <label htmlFor="showGPS" className="text-gray-700">Tampilkan GPS Raw</label>
            </div>

            <div className="flex items-center">
              <input
                type="checkbox"
                id="showIMU"
                checked={showIMU}
                onChange={(e) => setShowIMU(e.target.checked)}
                className="mr-2"
              />
              <label htmlFor="showIMU" className="text-gray-700">Tampilkan IMU Raw</label>
            </div>

            <div className="flex items-center">
              <input
                type="checkbox"
                id="showEKF"
                checked={showEKF}
                onChange={(e) => setShowEKF(e.target.checked)}
                className="mr-2"
              />
              <label htmlFor="showEKF" className="text-gray-700">Tampilkan Hasil EKF</label>
            </div>
          </div>

          <div className="mt-8">
            <h3 className="font-semibold mb-2">Status</h3>
            <div className="space-y-2 text-sm">
              <p>Posisi X: 123.45</p>
              <p>Posisi Y: 67.89</p>
              <p>Heading: 45°</p>
              <p>Kecepatan: 1.2 m/s</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SimulationPage;