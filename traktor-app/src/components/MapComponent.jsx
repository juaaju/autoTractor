import { MapContainer, TileLayer, Marker, Popup, Polygon } from "react-leaflet";
import "leaflet/dist/leaflet.css";

function MapComponent({ gpsData, tractorPosition, fieldCoords }) {
  const livePosition = [gpsData.latitude, gpsData.longitude];

  // Konversi posisi traktor menjadi array [lat, long] jika sudah diisi
  const tractorPos = tractorPosition.lat && tractorPosition.long
    ? [parseFloat(tractorPosition.lat), parseFloat(tractorPosition.long)]
    : null;

  // Konversi koordinat sawah ke dalam bentuk array lat-long
  const fieldPolygon = fieldCoords
    .filter((coord) => coord.lat && coord.long)
    .map((coord) => [parseFloat(coord.lat), parseFloat(coord.long)]);

  return (
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
    </MapContainer>
  );
}

export default MapComponent;
