import { useState, useEffect, useMemo } from "react";
import { MapContainer, TileLayer, Marker, Popup, Polygon, Polyline, useMap } from "react-leaflet";
import "leaflet/dist/leaflet.css";

// This component will handle updating the map view when coordinates change
// This component will only center the map on initial load
function MapUpdater({ center }) {
  const map = useMap();
  const [initialCenterSet, setInitialCenterSet] = useState(false);
  
  useEffect(() => {
    // Only center the map on valid coordinates for the first time
    if (!initialCenterSet && center && center[0] && center[1]) {
      map.setView(center);
      setInitialCenterSet(true);
    }
  }, [map, center, initialCenterSet]);
  
  return null;
}

function MapComponent({ 
  gpsData = {}, 
  imuData = {}, 
  ekfData = {}, 
  tractorPosition = {}, 
  fieldCoords = [], 
  zigzagPath = [] 
}) {
  const [gpsPath, setGpsPath] = useState([]);
  const [imuPath, setImuPath] = useState([]);
  const [ekfPath, setEkfPath] = useState([]);

  // Create positions with default fallbacks and validation
  const livePositionGps = useMemo(() => {
    return gpsData && gpsData.latitude && gpsData.longitude ? 
      [gpsData.latitude, gpsData.longitude] : 
      [0, 0]; // Default coordinates
  }, [gpsData]);

  const livePositionIMU = useMemo(() => {
    return imuData && imuData.latitude && imuData.longitude ? 
      [imuData.latitude, imuData.longitude] : 
      livePositionGps; // Fallback to GPS position
  }, [imuData, livePositionGps]);

  const livePositionEKF = useMemo(() => {
    return ekfData && ekfData.latitude && ekfData.longitude ? 
      [ekfData.latitude, ekfData.longitude] : 
      livePositionGps; // Fallback to GPS position
  }, [ekfData, livePositionGps]);

  // Determine if we have valid coordinates
  const hasValidCoordinates = useMemo(() => {
    return livePositionGps[0] !== 0 && livePositionGps[1] !== 0;
  }, [livePositionGps]);

  // Track GPS path
  useEffect(() => {
    if (gpsData && gpsData.latitude && gpsData.longitude) {
      setGpsPath((prevPath) => {
        // Only add new point if it's different from the last one
        const lastPoint = prevPath.length > 0 ? prevPath[prevPath.length - 1] : null;
        if (!lastPoint || 
            lastPoint[0] !== gpsData.latitude || 
            lastPoint[1] !== gpsData.longitude) {
          return [...prevPath, [gpsData.latitude, gpsData.longitude]];
        }
        return prevPath;
      });
    }
  }, [gpsData]);

  // Track IMU path
  useEffect(() => {
    if (imuData && imuData.latitude && imuData.longitude) {
      setImuPath((prevPath) => {
        const lastPoint = prevPath.length > 0 ? prevPath[prevPath.length - 1] : null;
        if (!lastPoint || 
            lastPoint[0] !== imuData.latitude || 
            lastPoint[1] !== imuData.longitude) {
          return [...prevPath, [imuData.latitude, imuData.longitude]];
        }
        return prevPath;
      });
    }
  }, [imuData]);

  // Track EKF path
  useEffect(() => {
    if (ekfData && ekfData.latitude && ekfData.longitude) {
      setEkfPath((prevPath) => {
        const lastPoint = prevPath.length > 0 ? prevPath[prevPath.length - 1] : null;
        if (!lastPoint || 
            lastPoint[0] !== ekfData.latitude || 
            lastPoint[1] !== ekfData.longitude) {
          return [...prevPath, [ekfData.latitude, ekfData.longitude]];
        }
        return prevPath;
      });
    }
  }, [ekfData]);

  // Safely parse tractor position
  const tractorPos = useMemo(() => {
    return tractorPosition && tractorPosition.lat && tractorPosition.long
      ? [parseFloat(tractorPosition.lat), parseFloat(tractorPosition.long)]
      : null;
  }, [tractorPosition]);

  // Safely parse field polygon
  const fieldPolygon = useMemo(() => {
    return fieldCoords
      .filter((coord) => coord && coord.lat && coord.long)
      .map((coord) => [parseFloat(coord.lat), parseFloat(coord.long)]);
  }, [fieldCoords]);

  // Check for valid zigzag path
  const validZigzagPath = useMemo(() => {
    return Array.isArray(zigzagPath) && zigzagPath.length > 0 && 
           zigzagPath.every(point => Array.isArray(point) && point.length === 2);
  }, [zigzagPath]);

  // Default center if no valid coordinates
  const initialCenter = hasValidCoordinates ? livePositionGps : [0, 0];
  
  return (
    <div className="bg-white p-6 rounded-lg shadow-lg">
      {!hasValidCoordinates && (
        <div className="bg-yellow-100 p-3 mb-4 rounded-md text-yellow-700">
          Waiting for valid GPS coordinates...
        </div>
      )}
      
      <MapContainer 
        center={initialCenter} 
        zoom={hasValidCoordinates ? 25 : 2} 
        style={{ height: "500px", width: "100%" }}
      >
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        
        {/* Map view updater */}
        <MapUpdater center={livePositionGps} />
        
        {/* Only render markers when we have valid coordinates */}
        {hasValidCoordinates && (
          <>
            {/* Marker posisi real-time gps */}
            <Marker position={livePositionGps}>
              <Popup>
                Lokasi Saat Ini GPS: {livePositionGps[0].toFixed(5)}, {livePositionGps[1].toFixed(5)}
              </Popup>
            </Marker>
            
            {/* Marker posisi real-time imu */}
            <Marker position={livePositionIMU}>
              <Popup>
                Lokasi Saat Ini IMU: {livePositionIMU[0].toFixed(5)}, {livePositionIMU[1].toFixed(5)}
              </Popup>
            </Marker>
            
            {/* Marker posisi real-time ekf */}
            <Marker position={livePositionEKF}>
              <Popup>
                Lokasi Saat Ini EKF: {livePositionEKF[0].toFixed(5)}, {livePositionEKF[1].toFixed(5)}
              </Popup>
            </Marker>
            
            {/* Tractor position marker */}
            {tractorPos && (
              <Marker position={tractorPos}>
                <Popup>Posisi Awal Traktor</Popup>
              </Marker>
            )}

            {/* Field area polygon */}
            {fieldPolygon.length === 4 && (
              <Polygon positions={fieldPolygon} color="blue">
                <Popup>Area Sawah</Popup>
              </Polygon>
            )}

            {/* GPS path tracking */}
            {gpsPath.length > 1 && (
              <Polyline positions={gpsPath} color="red" weight={3} />
            )}
            
            {/* IMU path tracking */}
            {imuPath.length > 1 && (
              <Polyline positions={imuPath} color="green" weight={3} />
            )}
            
            {/* EKF path tracking */}
            {ekfPath.length > 1 && (
              <Polyline positions={ekfPath} color="blue" weight={3} />
            )}
            
            {/* Zigzag Path Visualization */}
            {validZigzagPath && (
              <>
                {/* Visualisasi keseluruhan jalur */}
                <Polyline 
                  positions={zigzagPath} 
                  color="purple" 
                  weight={3} 
                  dashArray="5, 10" 
                />
                
                {/* Visualisasi setiap segmen jalur */}
                {Array.from({ length: Math.floor(zigzagPath.length / 2) }, (_, i) => {
                  const startIdx = i * 2;
                  const endIdx = startIdx + 1;
                  if (endIdx < zigzagPath.length) {
                    return (
                      <Polyline
                        key={`segment-${i}`}
                        positions={[zigzagPath[startIdx], zigzagPath[endIdx]]}
                        color={i % 2 === 0 ? "orange" : "teal"}
                        weight={4}
                      >
                        <Popup>Segmen {i + 1}</Popup>
                      </Polyline>
                    );
                  }
                  return null;
                })}
                
                {/* Marker untuk titik awal dan akhir jalur */}
                <Marker position={zigzagPath[0]}>
                  <Popup>Titik Awal Pembajakan</Popup>
                </Marker>
                
                <Marker position={zigzagPath[zigzagPath.length - 1]}>
                  <Popup>Titik Akhir Pembajakan</Popup>
                </Marker>
              </>
            )}
          </>
        )}
      </MapContainer>
      
      {/* Legend for paths */}
      <div className="mt-4 p-3 bg-gray-100 rounded-md">
        <h3 className="font-semibold mb-2">Keterangan Jalur</h3>
        <div className="grid grid-cols-2 gap-2">
          <div className="flex items-center mb-1">
            <div className="w-4 h-1 bg-red-500 mr-2"></div>
            <span>Jalur GPS</span>
          </div>
          <div className="flex items-center mb-1">
            <div className="w-4 h-1 bg-green-500 mr-2"></div>
            <span>Jalur IMU</span>
          </div>
          <div className="flex items-center mb-1">
            <div className="w-4 h-1 bg-blue-500 mr-2"></div>
            <span>Jalur EKF</span>
          </div>
          
          {validZigzagPath && (
            <>
              <div className="flex items-center mb-1">
                <div className="w-4 h-1 bg-purple-500 mr-2"></div>
                <span>Jalur Pembajakan Keseluruhan</span>
              </div>
              <div className="flex items-center mb-1">
                <div className="w-4 h-1 bg-orange-500 mr-2"></div>
                <span>Segmen Ganjil</span>
              </div>
              <div className="flex items-center mb-1">
                <div className="w-4 h-1 bg-teal-500 mr-2"></div>
                <span>Segmen Genap</span>
              </div>
            </>
          )}
        </div>
      </div>
    </div>
  );
}

export default MapComponent;