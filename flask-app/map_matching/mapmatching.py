import serial
import time
import pynmea2
import math
from typing import Optional, Tuple, List, Dict, Any

class GPSReader:
    def __init__(self, port: str, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect()
        
    def connect(self):
        try:
            if self.ser is None or not self.ser.is_open:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=0.5)
                print(f"Connected to GPS on {self.port}")
        except serial.SerialException as e:
            print(f"Failed to connect to GPS: {e}")
            self.ser = None
            
    def read(self) -> Optional[Tuple[float, float]]:
        """
        Membaca data GPS
        Returns:
            Tuple[float, float]: (latitude, longitude) jika berhasil
            None: jika gagal
        """
        try:
            if self.ser is None or not self.ser.is_open:
                self.connect()
                if self.ser is None:
                    return None
                    
            newdata = self.ser.readline()
            
            if newdata:
                newdata = newdata.decode('ascii', errors='ignore')
                
                if newdata.startswith("$GPGGA"):
                    try:
                        newmsg = pynmea2.parse(newdata)
                        lat = newmsg.latitude
                        lng = newmsg.longitude
                        return lat, lng
                    except pynmea2.nmea.ParseError as e:
                        print(f"Failed to parse NMEA sentence: {e}")
                        return None
                else:
                    return None
            else:
                print("No data received")
                return None
                
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.ser = None
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None
            
    def close(self):
        """Menutup koneksi serial"""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("GPS connection closed")


class MapMatcher:
    def __init__(self, path_coordinates: List[List[float]]):
        """
        Inisialisasi Map Matcher
        
        Args:
            path_coordinates: List koordinat [lat, lng] yang membentuk jalur
        """
        self.path = path_coordinates
        self.last_matched_idx = -1
        self.search_radius = 50  # Berapa banyak titik yang akan dicari di sekitar titik terakhir
        
    def haversine_distance(self, point1: List[float], point2: List[float]) -> float:
        """
        Menghitung jarak Haversine antara dua titik koordinat
        
        Args:
            point1: [lat, lng] koordinat pertama
            point2: [lat, lng] koordinat kedua
            
        Returns:
            float: Jarak dalam kilometer
        """
        R = 6371  # Radius bumi dalam km
        
        lat1, lon1 = point1
        lat2, lon2 = point2
        
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat / 2) * math.sin(dlat / 2) + 
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
             math.sin(dlon / 2) * math.sin(dlon / 2))
             
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        
        return distance
        
    def calculate_bearing(self, point1: List[float], point2: List[float]) -> float:
        """
        Menghitung arah/bearing antara dua titik
        
        Args:
            point1: [lat, lng] titik awal
            point2: [lat, lng] titik akhir
            
        Returns:
            float: Bearing dalam derajat (0-360)
        """
        lat1 = math.radians(point1[0])
        lon1 = math.radians(point1[1])
        lat2 = math.radians(point2[0])
        lon2 = math.radians(point2[1])
        
        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
             
        bearing = math.atan2(y, x)
        bearing = math.degrees(bearing)
        
        # Normalisasi ke 0-360
        return (bearing + 360) % 360
        
    def match_point(self, gps_point: List[float]) -> Dict[str, Any]:
        """
        Melakukan map matching untuk titik GPS ke jalur
        
        Args:
            gps_point: [lat, lng] koordinat dari GPS
            
        Returns:
            Dict: Informasi tentang hasil matching
        """
        # Tentukan jendela pencarian - jika ada titik yang cocok sebelumnya,
        # cari hanya di radius tertentu dari titik tersebut
        start_idx = 0
        end_idx = len(self.path) - 1
        
        if self.last_matched_idx >= 0:
            start_idx = max(0, self.last_matched_idx - 10)  # Cari 10 titik ke belakang
            end_idx = min(len(self.path) - 1, self.last_matched_idx + self.search_radius)
            
        # Cari titik terdekat dalam jendela pencarian
        min_dist = float('inf')
        closest_point = None
        closest_idx = -1
        
        for i in range(start_idx, end_idx + 1):
            dist = self.haversine_distance(self.path[i], gps_point)
            
            if dist < min_dist:
                min_dist = dist
                closest_point = self.path[i]
                closest_idx = i
                
        # Hitung persentase kemajuan di sepanjang jalur
        progress = closest_idx / (len(self.path) - 1)
        
        # Hitung arah/bearing jika memungkinkan
        bearing = None
        if closest_idx < len(self.path) - 1:
            bearing = self.calculate_bearing(self.path[closest_idx], self.path[closest_idx + 1])
            
        # Jarak dalam meter
        distance_to_path = min_dist * 1000
        
        # Perbarui indeks terakhir yang cocok
        self.last_matched_idx = closest_idx
        
        return {
            "matched_point": closest_point,
            "original_point": gps_point,
            "matched_index": closest_idx,
            "distance_to_path": distance_to_path,  # dalam meter
            "progress": progress,                  # 0 sampai 1 di sepanjang jalur
            "bearing": bearing,                    # arah jalur pada titik ini
            "is_on_path": distance_to_path < 10    # dianggap "di jalur" jika dalam radius 10 meter
        }


def generate_path_coordinates():
    """Menghasilkan koordinat jalur dari array1, array2, array3, dan array4"""
    c1 = -7.283564
    x1 = 112.796641
    x2 = 112.796465
    array1 = []
    epsilon = 1e-9  # toleransi untuk perbandingan float
    
    while abs(x1 - x2) > epsilon:
        array1.append([c1, round(x1, 6)])
        x1 -= 0.000001
        
    x1 = -7.283564
    x2 = -7.283510
    c1 = 112.796465
    array2 = []
    
    while abs(x1 - x2) > epsilon:
        array2.append([round(x1, 6), c1])
        x1 += 0.000001
        
    x1 = 112.796465
    x2 = 112.796641
    c1 = -7.283510
    array3 = []
    
    while abs(x1 - x2) > epsilon:
        array3.append([c1, round(x1, 6)])
        x1 += 0.000001
        
    x1 = -7.283510
    x2 = -7.283564
    c1 = 112.796641
    array4 = []
    
    while abs(x1 - x2) > epsilon:
        array4.append([round(x1, 6), c1])
        x1 -= 0.000001
        
    # Gabungkan semua array
    return array1 + array2 + array3 + array4


def main():
    # Generate jalur
    path_coordinates = generate_path_coordinates()
    print(f"Jumlah titik jalur: {len(path_coordinates)}")
    
    # Inisialisasi map matcher
    matcher = MapMatcher(path_coordinates)
    
    # Inisialisasi GPS reader
    gps = GPSReader(port="/dev/ttyACM0")
    
    try:
        while True:
            # Baca koordinat GPS
            coords = gps.read()
            
            if coords:
                lat, lng = coords
                
                # Lakukan map matching
                match_result = matcher.match_point([lat, lng])
                
                # Tampilkan informasi
                print(f"GPS: {lat}, {lng} | Matched: {match_result['matched_point']} | "
                      f"Distance: {match_result['distance_to_path']:.2f}m | "
                      f"Progress: {match_result['progress']:.2%} | "
                      f"On Path: {match_result['is_on_path']}")
                
            time.sleep(0.2)  # 5Hz update rate
            
    except KeyboardInterrupt:
        print("\nStopping GPS reader...")
    finally:
        gps.close()


if __name__ == "__main__":
    main()