import { useState, useEffect } from "react";
import { MapContainer, TileLayer, Marker, Popup, Polygon, Polyline } from "react-leaflet";
import "leaflet/dist/leaflet.css";

function MapComponent({ gpsData, tractorPosition, fieldCoords, zigzagPath = [] }) {
  const [path, setPath] = useState([]);

  const livePosition = [gpsData.latitude, gpsData.longitude];

  useEffect(() => {
    if (gpsData.latitude && gpsData.longitude) {
      setPath((prevPath) => [...prevPath, [gpsData.latitude, gpsData.longitude]]);
    }
  }, [gpsData]);

  // Konversi posisi traktor menjadi array [lat, long] jika sudah diisi
  const tractorPos = tractorPosition.lat && tractorPosition.long
    ? [parseFloat(tractorPosition.lat), parseFloat(tractorPosition.long)]
    : null;

  // Konversi koordinat sawah ke dalam bentuk array lat-long
  const fieldPolygon = fieldCoords
    .filter((coord) => coord.lat && coord.long)
    .map((coord) => [parseFloat(coord.lat), parseFloat(coord.long)]);

  // Validasi data zigzagPath
  const validZigzagPath = Array.isArray(zigzagPath) && zigzagPath.length > 0;
  
  return (
    <div className="bg-white p-6 rounded-lg shadow-lg">
      <MapContainer center={livePosition} zoom={25} style={{ height: "500px", width: "100%" }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        
        {/* Marker posisi real-time */}
        <Marker position={livePosition}>
          <Popup>
            Lokasi Saat Ini: {livePosition[0].toFixed(5)}, {livePosition[1].toFixed(5)}
          </Popup>
        </Marker>

        {/* Marker posisi awal traktor */}
        {tractorPos && (
          <Marker position={tractorPos}>
            <Popup>Posisi Awal Traktor</Popup>
          </Marker>
        )}

        {/* Poligon koordinat sawah */}
        {fieldPolygon.length === 4 && (
          <Polygon positions={fieldPolygon} color="blue">
            <Popup>Area Sawah</Popup>
          </Polygon>
        )}

        {/* Polyline untuk melacak jalur perjalanan */}
        {path.length > 1 && <Polyline positions={path} color="red" />}
        
        {/* Visualisasi jalur zigzag */}
        {validZigzagPath && (
          <>
            {/* Visualisasi keseluruhan jalur */}
            <Polyline 
              positions={zigzagPath} 
              color="green" 
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
                    color={i % 2 === 0 ? "blue" : "purple"}
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
      </MapContainer>
      
      {/* Legend untuk jalur */}
      {validZigzagPath && (
        <div className="mt-4 p-3 bg-gray-100 rounded-md">
          <h3 className="font-semibold mb-2">Keterangan Jalur</h3>
          <div className="flex items-center mb-1">
            <div className="w-4 h-1 bg-green-500 mr-2"></div>
            <span>Jalur Pembajakan Keseluruhan</span>
          </div>
          <div className="flex items-center mb-1">
            <div className="w-4 h-1 bg-blue-500 mr-2"></div>
            <span>Segmen Ganjil</span>
          </div>
          <div className="flex items-center mb-1">
            <div className="w-4 h-1 bg-purple-500 mr-2"></div>
            <span>Segmen Genap</span>
          </div>
          <div className="flex items-center">
            <div className="w-4 h-1 bg-red-500 mr-2"></div>
            <span>Jalur Traktor Aktual</span>
          </div>
        </div>
      )}
    </div>
  );
}

export default MapComponent;