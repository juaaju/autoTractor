import React, { useState, useEffect } from 'react';
import MapComponent from '../components/MapComponent'; // Import MapComponent (adjust the path if needed)

function MapPage() {
  const [gpsData, setGpsData] = useState({
    latitude: 51.505,  // Example latitude
    longitude: -0.09,  // Example longitude
  });

  const [tractorPosition, setTractorPosition] = useState({
    lat: 51.507,  // Example tractor latitude
    long: -0.08,  // Example tractor longitude
  });

  const [fieldCoords, setFieldCoords] = useState([
    { lat: 51.501, long: -0.12 },
    { lat: 51.503, long: -0.11 },
    { lat: 51.504, long: -0.13 },
    { lat: 51.502, long: -0.14 },
  ]);

  useEffect(() => {
    // Simulate dynamic GPS data update (this could come from an API or sensors)
    const interval = setInterval(() => {
      setGpsData({
        latitude: 51.505 + Math.random() * 0.01,  // Simulating GPS change
        longitude: -0.09 + Math.random() * 0.01,
      });
    }, 5000);

    return () => clearInterval(interval); // Cleanup on component unmount
  }, []);

  return (
    <div className="container mx-auto">
      <h1 className="text-center text-2xl font-semibold mb-4">Map of Tractor's Location</h1>
      <MapComponent gpsData={gpsData} tractorPosition={tractorPosition} fieldCoords={fieldCoords} />
    </div>
  );
}

export default MapPage;
