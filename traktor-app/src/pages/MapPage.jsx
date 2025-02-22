import React, { useState, useEffect } from 'react';
import MapComponent from '../components/MapComponent';
import SimulationPage from './SimulationPage';

function MapPage() {
  const [gpsData, setGpsData] = useState({
    latitude: 51.505,
    longitude: -0.09,
  });
  
  // const [tractorPosition] = useState({
  //   lat: 51.507,
  //   long: -0.08,
  // });
  
  // const [fieldCoords] = useState([
  //   { lat: 51.501, long: -0.12 },
  //   { lat: 51.503, long: -0.11 },
  //   { lat: 51.504, long: -0.13 },
  //   { lat: 51.502, long: -0.14 },
  // ]);

  const [tractorPosition, setTractorPosition] = useState({ lat: 0, long: 0 });
  const [fieldCoords, setFieldCoords] = useState([]);

  // Fungsi untuk mengambil data dari /data-form
  const fetchFormData = async () => {
    try {
      const response = await fetch("http://localhost:5000/data-form");
      const data = await response.json();

      console.log("Data fetched:", data); // Debugging

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

  useEffect(() => {
    fetchFormData();

    // Refresh data setiap 5 detik (opsional)
    const interval = setInterval(() => {
      fetchFormData();
    }, 5000);

    return () => clearInterval(interval); // Cleanup saat komponen di-unmount
  }, []);

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
    // Initial fetch
    fetchGpsData();

    // Set up polling interval
    const interval = setInterval(() => {
      fetchGpsData();
    }, 1000); // Poll every 5 seconds

    // Cleanup on component unmount
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="container mx-auto">
      <h1 className="flex font-bold m-4 text-3xl text-gray-800 justify-center">Peta Realtime </h1>
      <SimulationPage/>
      <MapComponent
        gpsData={gpsData} 
        tractorPosition={tractorPosition} 
        fieldCoords={fieldCoords} 
      />
      
    </div>
  );
}

export default MapPage;