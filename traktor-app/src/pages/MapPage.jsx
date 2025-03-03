import React, { useState, useEffect } from 'react';
import MapComponent from '../components/MapComponent';
import SimulationPage from './SimulationPage';

function MapPage() {
  const [gpsData, setGpsData] = useState({
    latitude: 51.505,
    longitude: -0.09,
  });
  
  const [imuData, setIMUData] = useState({
    latitude: 51.505,
    longitude: -0.09,
  });
  
  const [ekfData, setEKFData] = useState({
    latitude: 51.505,
    longitude: -0.09,
  });
  
  const [tractorPosition, setTractorPosition] = useState({ lat: 0, long: 0 });
  const [fieldCoords, setFieldCoords] = useState([]);
  const [zigzagPath, setZigzagPath] = useState([]);
  
  // Fetch form data
  const fetchFormData = async () => {
    try {
      const response = await fetch("http://localhost:5000/data-form");
      const data = await response.json();
      console.log("Data fetched:", data);
      
      if (data.zigzag_path && Array.isArray(data.zigzag_path)) {
        const formattedPath = data.zigzag_path.map(point => [
          point[0], 
          point[1]
        ]);
        setZigzagPath(formattedPath);
      }
      
      if (data.tractorPosition && typeof data.fieldPoints === "object") {
        setTractorPosition({
          lat: parseFloat(data.tractorPosition.latitude),
          long: parseFloat(data.tractorPosition.longitude),
        });
        
        const fieldArray = Object.values(data.fieldPoints).map(point => ({
          lat: parseFloat(point.latitude),
          long: parseFloat(point.longitude),
        }));
        setFieldCoords(fieldArray);
      } else {
        console.error("Invalid data format:", data);
      }
    } catch (error) {
      console.error("Error fetching data:", error);
    }
  };
  
  const fetchGpsData = async () => {
    try {
      const response = await fetch('http://localhost:5000/api/gps-dummy');
      const data = await response.json();
      setGpsData(data);
    } catch (error) {
      console.error('Error fetching GPS data:', error);
    }
  };
  
  const fetchIMUData = async () => {
    try {
      const response = await fetch('http://localhost:5001/data_imu');
      const data = await response.json();
      setIMUData(data);
    } catch (error) {
      console.error('Error fetching IMU data:', error);
    }
  };
  
  const fetchEKFData = async () => {
    try {
      const response = await fetch('http://localhost:5001/data_ekf');
      const data = await response.json();
      setEKFData(data);
    } catch (error) {
      console.error('Error fetching EKF data:', error);
    }
  };
  
  // Handle log file download
  const handleDownloadCSV = () => {
    // Assuming your sensor fusion server is running on port 5001
    window.open('http://localhost:5001/log_file', '_blank');
  };
  
  useEffect(() => {
    // Initial fetch for all data
    fetchGpsData();
    fetchIMUData();
    fetchEKFData();
    fetchFormData();
    
    // Set up polling intervals
    const gpsInterval = setInterval(fetchGpsData, 1000);
    const imuInterval = setInterval(fetchIMUData, 1000);
    const ekfInterval = setInterval(fetchEKFData, 1000);
    
    // Cleanup on component unmount
    return () => {
      clearInterval(gpsInterval);
      clearInterval(imuInterval);
      clearInterval(ekfInterval);
    };
  }, []);
  
  return (
    <div className="container mx-auto">
      <h1 className="flex font-bold m-4 text-3xl text-gray-800 justify-center">Peta Realtime</h1>
      <SimulationPage/>
      <MapComponent
        gpsData={gpsData} 
        imuData={imuData}
        ekfData={ekfData}
        tractorPosition={tractorPosition} 
        fieldCoords={fieldCoords}
        zigzagPath={zigzagPath}
      />
      <button
        onClick={handleDownloadCSV}
        type="button"
        className="px-6 m-4 py-2 bg-blue-600 text-white font-semibold rounded-lg shadow-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-opacity-50">
        Download CSV
      </button>
    </div>
  );
}

export default MapPage;