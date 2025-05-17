from math import radians, cos, sqrt
import time
import csv
import threading
from reqgps import ESP32GPSClient

# Mengasumsikan gpsread.py berisi class GPSReader
from gpsread import GPSReader

def latlon_to_xy(lat1, lon1, lat2, lon2):
    """Konversi dua koordinat GPS ke perbedaan x, y (meter)"""
    try:
        R = 6371000  # radius bumi dalam meter
        avg_lat = radians((lat1 + lat2) / 2.0)
        
        dx = radians(lon2 - lon1) * R * cos(avg_lat)
        dy = radians(lat2 - lat1) * R
        return dx, dy
    except Exception as e:
        print(f"Error in latlon_to_xy: {e}")
        return 0, 0

def distance(dx, dy):
    """Menghitung jarak dari komponen x dan y"""
    return sqrt(dx**2 + dy**2)

class GPSDataCollector:
    def __init__(self, csv_filename="gps_data.csv"):
        # Inisialisasi pembaca GPS
        print("Initializing GPS devices...")
        try:
            self.gps_rover = GPSReader(port="/dev/ttyACM0")
            print("Rover GPS initialized")
        except Exception as e:
            print(f"Error initializing rover GPS: {e}")
            self.gps_rover = None
        
        try:
            self.gps_base = ESP32GPSClient()
            print("Base GPS (ESP32) initialized")
        except Exception as e:
            print(f"Error initializing base GPS: {e}")
            self.gps_base = None
        
        # Data terbaru dari setiap GPS
        self.rover_data = None  # (lat, lon)
        self.base_data = None   # (lat, lon)
        
        # Status dan statistik
        self.rover_last_update = 0
        self.base_last_update = 0
        self.rover_total_readings = 0
        self.base_total_readings = 0
        
        # Lock untuk sinkronisasi thread
        self.rover_lock = threading.Lock()
        self.base_lock = threading.Lock()
        
        # CSV setup
        self.csv_filename = csv_filename
        self.setup_csv()
        
        # Kontrol thread
        self.running = False
        self.rover_thread = None
        self.base_thread = None
        self.status_thread = None
    
    def setup_csv(self):
        """Inisialisasi file CSV dengan header"""
        try:
            with open(self.csv_filename, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'base_lat', 'base_lon', 'rover_lat', 'rover_lon', 'x_diff', 'y_diff', 'distance']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
            print(f"CSV file initialized: {self.csv_filename}")
        except Exception as e:
            print(f"Error setting up CSV file: {e}")
    
    def read_rover_gps(self):
        """Thread function untuk membaca GPS rover"""
        if not self.gps_rover:
            print("Rover GPS not available. Thread exiting.")
            return
            
        print("Rover GPS reading thread started")
        while self.running:
            try:
                coords = self.gps_rover.read()
                if coords:
                    with self.rover_lock:
                        self.rover_data = coords
                        self.rover_last_update = time.time()
                        self.rover_total_readings += 1
                else:
                    # Jika tidak ada data, cetak pesan setiap 5 detik
                    now = time.time()
                    if now - self.rover_last_update > 5 and self.rover_last_update > 0:
                        print("No data from rover GPS for 5+ seconds")
                        self.rover_last_update = now  # Reset timer untuk pesan berikutnya
            except Exception as e:
                print(f"Error reading rover GPS: {e}")
            
            # Delay untuk mencegah polling yang terlalu cepat
            time.sleep(0.1)
    
    def read_base_gps(self):
        """Thread function untuk membaca GPS base dari ESP32"""
        if not self.gps_base:
            print("Base GPS not available. Thread exiting.")
            return
            
        print("Base GPS reading thread started")
        while self.running:
            try:
                lat, lon = self.gps_base.get_location(debug=False)
                if lat is not None and lon is not None:
                    with self.base_lock:
                        self.base_data = (lat, lon)
                        self.base_last_update = time.time()
                        self.base_total_readings += 1
                else:
                    # Jika tidak ada data, cetak pesan setiap 5 detik
                    now = time.time()
                    if now - self.base_last_update > 5 and self.base_last_update > 0:
                        print("No data from base GPS for 5+ seconds")
                        self.base_last_update = now  # Reset timer untuk pesan berikutnya
            except Exception as e:
                print(f"Error reading base GPS: {e}")
            
            # Delay untuk mencegah polling yang terlalu cepat
            time.sleep(0.1)
    
    def report_status(self):
        """Thread function untuk melaporkan status secara periodik"""
        print("Status reporting thread started")
        last_report_time = 0
        
        while self.running:
            now = time.time()
            # Laporan status setiap 10 detik
            if now - last_report_time >= 10:
                with self.rover_lock:
                    rover_data = self.rover_data
                    rover_readings = self.rover_total_readings
                
                with self.base_lock:
                    base_data = self.base_data
                    base_readings = self.base_total_readings
                
                print("\n--- GPS Status Report ---")
                print(f"Rover: {'Data available' if rover_data else 'No data'} - Total readings: {rover_readings}")
                print(f"Base: {'Data available' if base_data else 'No data'} - Total readings: {base_readings}")
                
                # Reset untuk pelaporan berikutnya
                last_report_time = now
            
            time.sleep(1)
    
    def start(self):
        """Memulai pengumpulan data dari kedua GPS secara paralel"""
        if not self.gps_rover and not self.gps_base:
            print("ERROR: Both GPS devices unavailable. Cannot start data collection.")
            return False
            
        self.running = True
        
        # Mulai thread untuk rover GPS
        if self.gps_rover:
            self.rover_thread = threading.Thread(target=self.read_rover_gps)
            self.rover_thread.daemon = True
            self.rover_thread.start()
        else:
            print("WARNING: Rover GPS unavailable - that thread will not start")
        
        # Mulai thread untuk base GPS
        if self.gps_base:
            self.base_thread = threading.Thread(target=self.read_base_gps)
            self.base_thread.daemon = True
            self.base_thread.start()
        else:
            print("WARNING: Base GPS unavailable - that thread will not start")
        
        # Thread untuk status reporting
        self.status_thread = threading.Thread(target=self.report_status)
        self.status_thread.daemon = True
        self.status_thread.start()
        
        print("GPS data collection started.")
        return True
    
    def stop(self):
        """Menghentikan pengumpulan data"""
        self.running = False
        
        if self.rover_thread:
            self.rover_thread.join(2.0)  # Wait max 2 seconds
        
        if self.base_thread:
            self.base_thread.join(2.0)  # Wait max 2 seconds
            
        if self.status_thread:
            self.status_thread.join(2.0)  # Wait max 2 seconds
            
        print("GPS data collection stopped.")
    
    def process_data(self):
        """Memproses data terbaru dan menyimpan ke CSV"""
        # Mengambil data terbaru dengan thread safety
        with self.rover_lock:
            rover_data = self.rover_data
            
        with self.base_lock:
            base_data = self.base_data
        
        # Hanya memproses jika kedua GPS memiliki data valid
        if rover_data and base_data:
            try:
                lat1, lon1 = base_data
                lat2, lon2 = rover_data
                
                # Hitung perbedaan x, y (meter)
                x_diff, y_diff = latlon_to_xy(lat1, lon1, lat2, lon2)
                dist = distance(x_diff, y_diff)
                
                # Simpan ke CSV
                timestamp = time.time()
                self.save_to_csv(timestamp, lat1, lon1, lat2, lon2, x_diff, y_diff, dist)
                
                return {
                    'timestamp': timestamp,
                    'base': base_data,
                    'rover': rover_data,
                    'diff': (x_diff, y_diff),
                    'distance': dist
                }
            except Exception as e:
                print(f"Error processing GPS data: {e}")
        else:
            # Pesan lebih informatif ketika data tidak tersedia
            missing = []
            if not base_data:
                missing.append("base")
            if not rover_data:
                missing.append("rover")
                
            # Pesan ditampilkan hanya sekali setiap 5 detik untuk menghindari spam
            if time.time() % 5 < 0.1:
                print(f"Waiting for GPS data from: {', '.join(missing)}")
        
        return None
    
    def save_to_csv(self, timestamp, base_lat, base_lon, rover_lat, rover_lon, x_diff, y_diff, distance):
        """Menyimpan satu baris data ke CSV"""
        try:
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=[
                    'timestamp', 'base_lat', 'base_lon', 'rover_lat', 'rover_lon', 'x_diff', 'y_diff', 'distance'
                ])
                writer.writerow({
                    'timestamp': timestamp,
                    'base_lat': base_lat,
                    'base_lon': base_lon,
                    'rover_lat': rover_lat,
                    'rover_lon': rover_lon,
                    'x_diff': x_diff,
                    'y_diff': y_diff,
                    'distance': distance
                })
        except Exception as e:
            print(f"Error saving to CSV: {e}")

def main():
    """Fungsi utama program"""
    try:
        # Buat collector dan mulai pengumpulan data
        collector = GPSDataCollector(csv_filename="gps_diff_data.csv")
        if not collector.start():
            print("Failed to start GPS data collection. Exiting.")
            return
        
        print("Program running. Press Ctrl+C to exit.")
        
        # Main loop - memproses dan menampilkan data
        while True:
            data = collector.process_data()
            if data:
                print(f"\nBase: {data['base'][0]:.6f}, {data['base'][1]:.6f}")
                print(f"Rover: {data['rover'][0]:.6f}, {data['rover'][1]:.6f}")
                print(f"Diff: {data['diff'][0]:.2f}m, {data['diff'][1]:.2f}m")
                print(f"Distance: {data['distance']:.2f}m")
                print("-" * 30)
            
            time.sleep(1)  # Proses data setiap detik
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Pastikan pengumpulan data dihentikan dengan benar
        if 'collector' in locals():
            collector.stop()
        print("Program terminated.")

if __name__ == "__main__":
    main()
