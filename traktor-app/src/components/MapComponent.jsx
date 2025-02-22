import { useState, useEffect } from "react";
import { MapContainer, TileLayer, Marker, Popup, Polygon, Polyline } from "react-leaflet";
import "leaflet/dist/leaflet.css";

function MapComponent({ gpsData, tractorPosition, fieldCoords }) {
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
      </MapContainer>
    </div>
  );
}

export default MapComponent;
