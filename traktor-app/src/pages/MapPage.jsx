import React, { useState, useEffect } from 'react';
import MapComponent from '../components/MapComponent';
import SimulationPage from './SimulationPage';

function MapPage() {
  const [gpsData, setGpsData] = useState({
    latitude: 51.505,
    longitude: -0.09,
  });
  const [tractorPosition, setTractorPosition] = useState({ lat: 0, long: 0 });
  const [fieldCoords, setFieldCoords] = useState([]);
  const [zigzagPath, setZigzagPath] = useState([]); // Tambahkan state untuk zigzagPath
  
  // Fungsi untuk mengambil data dari /data-form
  const fetchFormData = async () => {
    try {
      const response = await fetch("http://localhost:5000/data-form");
      const data = await response.json();
      console.log("Data fetched:", data); // Debugging
      
      // Tambahkan pemrosesan data zigzag path
      if (data.zigzag_path && Array.isArray(data.zigzag_path)) {
        // Format zigzag_path menjadi format yang dibutuhkan oleh Leaflet
        const formattedPath = data.zigzag_path.map(point => [
          point[0], // latitude 
          point[1]  // longitude
        ]);
        setZigzagPath(formattedPath);
      }
      
      if (data.tractorPosition && typeof data.fieldPoints === "object") {
        setTractorPosition({
          lat: parseFloat(data.tractorPosition.latitude),
          long: parseFloat(data.tractorPosition.longitude),
        });
        // Ubah fieldPoints (object) jadi array
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
      const response = await fetch('http://localhost:5000/api/gps-data');
      const data = await response.json();
      setGpsData(data);
    } catch (error) {
      console.error('Error fetching GPS data:', error);
    }
  };
  
  useEffect(() => {
    // Initial fetch GPS data
    fetchGpsData();
    
    // Initial fetch form data
    fetchFormData(); // Tambahkan pemanggilan fetchFormData
    
    // Set up polling interval for GPS
    const interval = setInterval(() => {
      fetchGpsData();
    }, 1000);
    
    // Cleanup on component unmount
    return () => clearInterval(interval);
  }, []);
  
  return (
    <div className="container mx-auto">
      <h1 className="flex font-bold m-4 text-3xl text-gray-800 justify-center">Peta Realtime </h1>
      <SimulationPage/>
      {/* MapComponent dengan zigzagPath sebagai prop */}
      <MapComponent
        gpsData={gpsData} 
        tractorPosition={tractorPosition} 
        fieldCoords={fieldCoords}
        zigzagPath={zigzagPath}
      />
    </div>
  );
}

export default MapPage;