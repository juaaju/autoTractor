import requests
import time
import re

class ESP32GPSClient:
    """
    Class untuk mengakses dan memproses data GPS dari ESP32
    """
    
    def __init__(self, esp32_url="http://192.168.4.1", request_timeout=5):
        """
        Inisialisasi ESP32GPSClient
        
        Args:
            esp32_url: URL ESP32 yang menjalankan server GPS
            request_timeout: Timeout untuk request HTTP dalam detik
        """
        self.esp32_url = esp32_url
        self.request_timeout = request_timeout
        self.last_valid_location = None  # (lat, lon) terakhir yang valid
    
    def parse_nmea_latitude_longitude(self, sentence):
        """
        Parse latitude dan longitude dari kalimat NMEA
        
        Args:
            sentence: String kalimat NMEA
            
        Returns:
            (latitude, longitude) dalam format derajat desimal atau (None, None) jika gagal
        """
        try:
            parts = sentence.split(',')
            
            # Format GPRMC: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
            if sentence.startswith('$GPRMC'):
                if len(parts) >= 7 and parts[2] == 'A':  # 'A' berarti data valid
                    # Parse latitude
                    lat = float(parts[3][0:2]) + float(parts[3][2:]) / 60.0
                    if parts[4] == 'S':
                        lat = -lat
                    
                    # Parse longitude
                    lon = float(parts[5][0:3]) + float(parts[5][3:]) / 60.0
                    if parts[6] == 'W':
                        lon = -lon
                    
                    return (lat, lon)
            
            # Format GPGGA: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
            elif sentence.startswith('$GPGGA'):
                if len(parts) >= 7 and parts[6] != '0':  # parts[6] adalah fix quality, '0' berarti no fix
                    # Parse latitude
                    lat = float(parts[2][0:2]) + float(parts[2][2:]) / 60.0
                    if parts[3] == 'S':
                        lat = -lat
                    
                    # Parse longitude
                    lon = float(parts[4][0:3]) + float(parts[4][3:]) / 60.0
                    if parts[5] == 'W':
                        lon = -lon
                    
                    return (lat, lon)
            
            # Format GPGLL: $GPGLL,4916.45,N,12311.12,W,225444,A,*1D
            elif sentence.startswith('$GPGLL'):
                if len(parts) >= 7 and parts[6] == 'A':  # 'A' berarti data valid
                    # Parse latitude
                    lat = float(parts[1][0:2]) + float(parts[1][2:]) / 60.0
                    if parts[2] == 'S':
                        lat = -lat
                    
                    # Parse longitude
                    lon = float(parts[3][0:3]) + float(parts[3][3:]) / 60.0
                    if parts[4] == 'W':
                        lon = -lon
                    
                    return (lat, lon)
        
        except (ValueError, IndexError) as e:
            if self.debug:
                print(f"Error parsing NMEA sentence: {e}")
        
        return (None, None)
    
    def get_raw_gps_data(self):
        """
        Mengambil data mentah GPS dari ESP32
        
        Returns:
            dict: Data mentah dari ESP32 atau None jika gagal
        """
        try:
            response = requests.get(self.esp32_url, timeout=self.request_timeout)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Error: Status kode HTTP {response.status_code}")
        
        except requests.exceptions.RequestException as e:
            print(f"Error koneksi: {e}")
        except ValueError as e:
            print(f"Error parsing JSON: {e}")
        except Exception as e:
            print(f"Error tidak terduga: {e}")
        
        return None
    
    def get_location(self, debug=False):
        """
        Mengambil lokasi GPS (latitude, longitude) dari ESP32
        
        Args:
            debug: Boolean untuk menampilkan informasi debug
            
        Returns:
            tuple: (latitude, longitude) atau (None, None) jika gagal
        """
        self.debug = debug
        
        try:
            # Dapatkan data mentah
            data = self.get_raw_gps_data()
            
            if not data:
                return None, None
            
            # Ambil array data NMEA
            nmea_data = data.get("nmea_data", [])
            
            if debug:
                print(f"Ditemukan {len(nmea_data)} kalimat NMEA")
            
            # Cari kalimat yang dimulai dengan $GPRMC, $GPGGA, atau $GPGLL
            for sentence in nmea_data:
                if sentence.startswith("$GPRMC") or sentence.startswith("$GPGGA") or sentence.startswith("$GPGLL"):
                    lat, lon = self.parse_nmea_latitude_longitude(sentence)
                    if lat is not None and lon is not None:
                        if debug:
                            print(f"Kalimat NMEA: {sentence}")
                            print(f"Latitude: {lat:.6f}°")
                            print(f"Longitude: {lon:.6f}°")
                        
                        # Simpan lokasi valid terakhir
                        self.last_valid_location = (lat, lon)
                        return lat, lon
            
            if debug and nmea_data:
                # Tampilkan info status dari kalimat NMEA untuk debugging
                for sentence in nmea_data:
                    parts = sentence.split(',')
                    
                    if sentence.startswith("$GPRMC") and len(parts) > 2:
                        print(f"GPRMC status: {parts[2]} ({'Valid' if parts[2] == 'A' else 'Not valid'})")
                    
                    elif sentence.startswith("$GPGGA") and len(parts) > 6:
                        print(f"GPGGA fix quality: {parts[6]} ({'Fix' if parts[6] != '0' else 'No fix'})")
                    
                    elif sentence.startswith("$GPGLL") and len(parts) > 6:
                        print(f"GPGLL status: {parts[6]} ({'Valid' if parts[6] == 'A' else 'Not valid'})")
            
            if debug:
                print("Tidak ditemukan data GPS yang valid dalam respons JSON")
        
        except Exception as e:
            if debug:
                print(f"Error dalam get_location: {e}")
        
        return None, None
    
    def get_last_valid_location(self):
        """
        Mendapatkan lokasi valid terakhir yang diketahui
        
        Returns:
            tuple: (latitude, longitude) atau (None, None) jika belum ada lokasi valid
        """
        return self.last_valid_location
    
    def monitor_gps(self, callback=None, interval=1.0, debug=False):
        """
        Memantau data GPS secara terus menerus
        
        Args:
            callback: Fungsi yang akan dipanggil dengan (lat, lon) saat lokasi valid
            interval: Interval polling dalam detik
            debug: Boolean untuk menampilkan informasi debug
        """
        try:
            print("Monitoring GPS data dari ESP32...")
            print("Tekan Ctrl+C untuk keluar")
            
            while True:
                lat, lon = self.get_location(debug=debug)
                
                if lat is not None and lon is not None:
                    if callback:
                        callback(lat, lon)
                
                time.sleep(interval)
        
        except KeyboardInterrupt:
            print("\nMonitoring GPS dihentikan oleh pengguna")


# Jika file ini dijalankan langsung (bukan diimpor)
if __name__ == "__main__":
    # Contoh penggunaan class
    def display_location(lat, lon):
        print(f"Lokasi saat ini: {lat:.6f}°, {lon:.6f}°")
    
    # Buat instance dari ESP32GPSClient
    gps_client = ESP32GPSClient()
    
    # Pantau GPS dengan callback yang menampilkan lokasi
    gps_client.monitor_gps(callback=display_location, interval=1.0, debug=True)