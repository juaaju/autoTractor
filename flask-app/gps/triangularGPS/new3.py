import numpy as np
import time
import threading
from queue import Queue
from collections import deque
import datetime
from scipy.optimize import minimize
from gpsread import GPSReader  # Import class GPSReader dari modul gpsread
import pymap3d as pm
import csv
import os

# Referensi koordinat - UPDATE dengan koordinat GPS aktual
LAT_REF = -7.283590  # Gunakan lat dari GPS1 sebagai referensi
LNG_REF = 112.796401  # Gunakan lon dari GPS1 sebagai referensi
ALT_REF = 0  # Altitude default untuk referensi

# Mode untuk meningkatkan akurasi GPS murah dengan multiple GPS
MULTI_GPS_MODE = True  # Set True untuk multiple GPS averaging
SIMULATION_MODE = False  # Set True hanya untuk testing algorithm

# Triangle referensi relatif (posisi fisik GPS di robot dalam meter)
# GPS dipasang dalam triangle equilateral 30cm di robot
RDT_PHYSICAL = np.array([
    [0.0, 0.0],                    # GPS 1 - pojok kiri bawah  
    [0.3, 0.0],                    # GPS 2 - pojok kanan bawah (30cm ke kanan)
    [0.15, 0.3 * np.sqrt(3)/2]     # GPS 3 - pojok atas (triangle equilateral)
])

print(f"Konfigurasi fisik GPS triangle (dalam meter):")
for i, pos in enumerate(RDT_PHYSICAL):
    print(f"  GPS{i+1}: [{pos[0]:.3f}, {pos[1]:.3f}]")

# Class untuk membaca GPS secara asinkron
class AsyncGPSReader:
    def __init__(self, port, buffer_size=10, timeout=1.0):
        self.gps = GPSReader(port=port)
        self.buffer = deque(maxlen=buffer_size)
        self.last_position = None
        self.lock = threading.Lock()
        self.running = True
        self.timeout = timeout
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def _read_loop(self):
        while self.running:
            try:
                position = self.gps.read()  # Menggunakan method read() yang benar
                if position is not None:  # Hanya simpan jika ada data valid
                    timestamp = time.time()
                    
                    with self.lock:
                        self.last_position = position
                        self.buffer.append((timestamp, position))
            except Exception as e:
                print(f"Error membaca GPS pada {self.gps.port}: {e}")
            
            time.sleep(0.1)  # Polling lebih lambat untuk mengurangi beban CPU
    
    def get_latest_position(self):
        with self.lock:
            if self.last_position is None:
                raise ValueError("Belum ada data GPS")
            return self.last_position
    
    def get_position_at_time(self, target_time, max_age=1.0):
        """Mendapatkan posisi terdekat dengan timestamp target"""
        with self.lock:
            if not self.buffer:
                raise ValueError("Buffer GPS kosong")
            
            # Mencari data dengan timestamp terdekat
            closest = min(self.buffer, key=lambda x: abs(x[0] - target_time))
            
            # Periksa apakah data tidak terlalu lama
            if abs(closest[0] - target_time) > max_age:
                raise ValueError(f"Data GPS terlalu lama (umur: {abs(closest[0] - target_time):.2f}s)")
            
            return closest[1]  # Mengembalikan posisi (lat, lon)
    
    def stop(self):
        self.running = False
        if self.gps.ser is not None:
            self.gps.close()  # Tutup koneksi serial GPS
        self.thread.join(timeout=1.0)

# Class untuk logging data ke CSV
class CSVLogger:
    def __init__(self, filename=None):
        if filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"gps_data_{timestamp}.csv"
        
        self.filename = filename
        self.file_handle = None
        self.writer = None
        self.lock = threading.Lock()
        self._init_csv()
    
    def _init_csv(self):
        """Inisialisasi file CSV dengan header"""
        file_exists = os.path.exists(self.filename)
        self.file_handle = open(self.filename, 'a', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file_handle)
        
        if not file_exists:
            # Tulis header jika file baru
            header = [
                'timestamp', 'datetime',
                'gps1_lat', 'gps1_lon', 'gps1_x', 'gps1_y',
                'gps2_lat', 'gps2_lon', 'gps2_x', 'gps2_y',
                'gps3_lat', 'gps3_lon', 'gps3_x', 'gps3_y',
                'estimated_x', 'estimated_y',
                'smoothed_x', 'smoothed_y'
            ]
            self.writer.writerow(header)
            self.file_handle.flush()
    
    def log_data(self, timestamp, gps_latlon_data, gps_xy_data, estimated_pos, smoothed_pos):
        """Simpan data ke CSV"""
        with self.lock:
            try:
                dt_string = datetime.datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")
                
                row = [timestamp, dt_string]
                
                # Data GPS (lat, lon, x, y untuk setiap GPS)
                for i in range(3):
                    if i < len(gps_latlon_data) and i < len(gps_xy_data):
                        lat, lon = gps_latlon_data[i]
                        x, y = gps_xy_data[i]
                        row.extend([lat, lon, x, y])
                    else:
                        # Jika data GPS tidak tersedia
                        row.extend([None, None, None, None])
                
                # Posisi estimasi dan smoothed
                row.extend([estimated_pos[0], estimated_pos[1]])
                row.extend([smoothed_pos[0], smoothed_pos[1]])
                
                self.writer.writerow(row)
                self.file_handle.flush()
                
            except Exception as e:
                print(f"Error saat menyimpan ke CSV: {e}")
    
    def close(self):
        """Tutup file CSV"""
        with self.lock:
            if self.file_handle:
                self.file_handle.close()

def latlon_to_xy(lat, lon, lat_ref, lon_ref, alt_ref=0):
    # Gunakan altitude 0 sebagai default jika tidak tersedia
    alt = 0
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
    return e, n  # east (x), north (y)

def compute_centroid(points):
    return np.mean(points, axis=0)

def center_points(points):
    centroid = compute_centroid(points)
    return points - centroid

def rotate_points(points, theta):
    rot = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta),  np.cos(theta)]])
    return points @ rot.T

def rotation_loss(theta, ref_centered, gps_centered):
    rotated = rotate_points(gps_centered, theta)
    return np.sum(np.linalg.norm(rotated - ref_centered, axis=1))

def multi_gps_positioning(gps_xy_data):
    """
    Improved positioning menggunakan multiple GPS untuk akurasi lebih baik
    
    Args:
        gps_xy_data: Array koordinat XY dari multiple GPS
    
    Returns:
        improved_position: Posisi dengan akurasi yang ditingkatkan
        confidence: Confidence score (0-1)
    """
    if len(gps_xy_data) < 2:
        return None, 0.0
    
    # Method 1: Simple averaging (baseline)
    avg_position = compute_centroid(gps_xy_data)
    
    # Method 2: Weighted averaging berdasarkan konsistensi
    if len(gps_xy_data) >= 3:
        # Hitung jarak dari setiap GPS ke centroid
        distances_to_center = [np.linalg.norm(pos - avg_position) for pos in gps_xy_data]
        max_dist = max(distances_to_center)
        
        if max_dist > 0:
            # Weight berdasarkan kedekatan ke centroid (GPS yang konsisten dapat weight lebih)
            weights = [(max_dist - dist) / max_dist + 0.1 for dist in distances_to_center]  # +0.1 untuk minimum weight
            weights = np.array(weights) / sum(weights)  # Normalize
            
            # Weighted average
            weighted_position = np.average(gps_xy_data, weights=weights, axis=0)
        else:
            weighted_position = avg_position
        
        # Confidence berdasarkan konsistensi GPS
        std_dev = np.std(distances_to_center)
        confidence = max(0.0, min(1.0, 1.0 - std_dev/5.0))  # Scale 0-1
        
        return weighted_position, confidence
    else:
        # Hanya 2 GPS
        return avg_position, 0.7

def estimate_robot_center(gps_xy_data):
    """
    Estimasi posisi pusat robot berdasarkan multiple GPS dan konfigurasi fisik
    """
    if len(gps_xy_data) < 3:
        # Jika kurang dari 3 GPS, gunakan simple averaging
        position, confidence = multi_gps_positioning(gps_xy_data)
        return position, confidence, "simple_avg"
    
    # Metode 1: Multi-GPS averaging (untuk akurasi)
    improved_pos, confidence = multi_gps_positioning(gps_xy_data)
    
    # Metode 2: Geometric center compensation
    # Karena GPS tersebar dalam triangle 30cm, pusat robot bukan di centroid GPS
    # tapi di geometric center dari triangle
    
    # Hitung centroid GPS yang terdeteksi
    gps_centroid = compute_centroid(gps_xy_data)
    
    # Estimasi orientasi robot dari pola GPS (opsional, untuk future improvement)
    # Untuk sekarang, asumsikan robot center = GPS centroid
    
    # Gabungkan kedua metode dengan weighting
    alpha = confidence  # Semakin tinggi confidence, semakin percaya improved_pos
    final_position = alpha * improved_pos + (1 - alpha) * gps_centroid
    
    return final_position, confidence, "multi_gps"

# Inisialisasi GPS readers asinkron
gps1 = AsyncGPSReader(port="/dev/ttyACM0")
gps2 = AsyncGPSReader(port="/dev/ttyUSB0")
gps3 = AsyncGPSReader(port="/dev/ttyUSB1")

# Kumpulan GPS readers
gps_readers = [gps1, gps2, gps3]

# Auto-calibration untuk koordinat referensi
def auto_calibrate_reference():
    """Auto-calibrate koordinat referensi berdasarkan GPS aktual"""
    global LAT_REF, LNG_REF
    
    print("Auto-calibrating koordinat referensi...")
    time.sleep(3)  # Tunggu GPS stabilize
    
    total_lat, total_lon = 0, 0
    valid_count = 0
    
    for i, gps in enumerate(gps_readers):
        try:
            position = gps.get_latest_position()
            if position is not None:
                lat, lon = position
                total_lat += lat
                total_lon += lon
                valid_count += 1
                print(f"GPS{i+1}: lat={lat:.6f}, lon={lon:.6f}")
        except:
            pass
    
    if valid_count > 0:
        LAT_REF = total_lat / valid_count
        LNG_REF = total_lon / valid_count
        print(f"Koordinat referensi diupdate: lat={LAT_REF:.6f}, lon={LNG_REF:.6f}")
    else:
        print("Gagal auto-calibrate, menggunakan koordinat referensi default")

# Panggil auto-calibration
auto_calibrate_reference()

# Referensi koordinat RDT (Robot Desired Triangle) - TIDAK DIGUNAKAN lagi
# Sekarang kita gunakan pendekatan multi-GPS averaging yang berbeda

def get_gps_positions(max_age=2.0):  # Naikkan default max_age
    """Mendapatkan posisi dari semua GPS pada waktu yang hampir bersamaan"""
    global SIMULATION_MODE
    
    if SIMULATION_MODE:
        # Mode simulasi: buat data GPS sintetis dengan noise kecil
        base_lat = LAT_REF
        base_lon = LNG_REF
        
        # Noise kecil untuk simulasi variasi GPS (±1 meter dalam lat/lon)
        noise_scale = 0.00001  # ≈ 1 meter
        
        # Buat 3 GPS dengan posisi yang berbeda dalam range 30cm
        import random
        
        gps_positions_latlon = []
        gps_positions_xy = []
        
        for i in range(3):
            # Tambahkan noise kecil untuk setiap GPS
            lat = base_lat + random.uniform(-noise_scale, noise_scale)
            lon = base_lon + random.uniform(-noise_scale, noise_scale)
            
            xy = latlon_to_xy(lat, lon, LAT_REF, LNG_REF, ALT_REF)
            
            gps_positions_latlon.append((lat, lon))
            gps_positions_xy.append(xy)
            
            print(f"GPS{i+1}: Fresh data (SIM) - lat={lat:.6f}, lon={lon:.6f}")
        
        return gps_positions_latlon, np.array(gps_positions_xy)
    
    else:
        # Mode real GPS - kode asli
        now = time.time()
        
        positions_latlon = []
        positions_xy = []
        
        for i, gps in enumerate(gps_readers):
            try:
                position = gps.get_position_at_time(now, max_age)
                if position is not None:  # Pastikan data valid
                    lat, lon = position
                    xy = latlon_to_xy(lat, lon, LAT_REF, LNG_REF, ALT_REF)
                    positions_latlon.append((lat, lon))
                    positions_xy.append(xy)
                    print(f"GPS{i+1}: Fresh data - lat={lat:.6f}, lon={lon:.6f}")
                else:
                    raise ValueError("Data GPS kosong")
            except Exception as e:
                # Jika data tidak tersedia, gunakan data terakhir yang valid
                try:
                    position = gps.get_latest_position()
                    if position is not None:
                        lat, lon = position
                        xy = latlon_to_xy(lat, lon, LAT_REF, LNG_REF, ALT_REF)
                        positions_latlon.append((lat, lon))
                        positions_xy.append(xy)
                        print(f"GPS{i+1}: Using old data - {e}")
                    else:
                        raise ValueError("Tidak ada data GPS valid")
                except Exception as e2:
                    # Jika tidak ada data valid
                    print(f"GPS{i+1}: No valid data - {e2}")
                    positions_latlon.append((None, None))
                    positions_xy.append((None, None))
        
        # Filter out invalid positions (None values)
        valid_positions_xy = [pos for pos in positions_xy if pos[0] is not None and pos[1] is not None]
        
        return positions_latlon, np.array(valid_positions_xy) if valid_positions_xy else np.array([])

# Inisialisasi CSV logger
csv_logger = CSVLogger()

# Membuat buffer untuk posisi terakhir, untuk smoothing
position_buffer = deque(maxlen=5)

try:
    print("Memulai Multi-GPS Tracking...")
    print(f"Referensi koordinat: {LAT_REF}, {LNG_REF}")
    print(f"Konfigurasi fisik: 3 GPS dalam triangle 30cm")
    print(f"Tujuan: Meningkatkan akurasi GPS murah melalui averaging dan triangulation")
    print(f"Data akan disimpan ke: {csv_logger.filename}")
    print("Tekan Ctrl+C untuk menghentikan...")
    print("=" * 60)
    
    # Main loop
    while True:
        try:
            current_time = time.time()
            
            # Baca posisi GPS dan konversi ke koordinat kartesian
            gps_latlon_data, GDT = get_gps_positions(max_age=2.0)  # Naikkan tolerance
            
            if len(GDT) >= 2:  # Minimal 2 GPS untuk positioning
                # Debug: Print raw GPS coordinates dan jarak antar GPS
                print(f"Raw GPS coordinates:")
                for i, xy in enumerate(GDT):
                    print(f"  GPS{i+1}: [{xy[0]:.6f}, {xy[1]:.6f}]")
                
                if len(GDT) >= 3:
                    # Hitung jarak antar GPS untuk validasi
                    dist_12 = np.linalg.norm(GDT[1] - GDT[0])
                    dist_13 = np.linalg.norm(GDT[2] - GDT[0]) 
                    dist_23 = np.linalg.norm(GDT[2] - GDT[1])
                    print(f"Jarak GPS: 1-2={dist_12:.3f}m, 1-3={dist_13:.3f}m, 2-3={dist_23:.3f}m")
                    
                    # Adaptive tolerance - untuk GPS consumer, jarak 3-10m masih wajar
                    distances = [dist_12, dist_13, dist_23]
                    avg_distance = np.mean(distances)
                    std_distance = np.std(distances)
                    
                    print(f"Rata-rata jarak: {avg_distance:.3f}m ± {std_distance:.3f}m")
                    
                    if avg_distance > 15.0:
                        print("⚠️  WARNING: Jarak antar GPS sangat besar (>15m)")
                    elif avg_distance < 1.0:
                        print("✅ GPS sangat dekat - ideal untuk precision")  
                    else:
                        print("📍 Jarak GPS dalam range normal untuk GPS consumer")
                
                # Gunakan multi-GPS positioning untuk akurasi lebih baik
                pos_estimated, confidence, method = estimate_robot_center(GDT)
                
                print(f"Method: {method}, Confidence: {confidence:.2f}")
                print(f"Estimated position: [{pos_estimated[0]:.3f}, {pos_estimated[1]:.3f}]")
                
                # Tambahkan ke buffer untuk smoothing
                position_buffer.append(pos_estimated)
                
                # Posisi hasil smoothing (rata-rata dari beberapa posisi terakhir)
                smoothed_position = np.mean(position_buffer, axis=0) if position_buffer else pos_estimated
                
                # Simpan data ke CSV
                csv_logger.log_data(current_time, gps_latlon_data, GDT, pos_estimated, smoothed_position)
                
                # Output yang lebih informatif
                print(f"GPS: {len(GDT)}/3 | " +
                      f"Est=[{pos_estimated[0]:.3f}, {pos_estimated[1]:.3f}] " +
                      f"Smooth=[{smoothed_position[0]:.3f}, {smoothed_position[1]:.3f}] " +
                      f"Conf={confidence:.2f} Buf={len(position_buffer)}")
                print("-" * 60)
            elif len(GDT) > 0:
                # Jika hanya ada 1-2 GPS, gunakan centroid sederhana tanpa procrustes
                pos_estimated = compute_centroid(GDT)
                position_buffer.append(pos_estimated)
                smoothed_position = np.mean(position_buffer, axis=0) if position_buffer else pos_estimated
                
                # Simpan data ke CSV (meskipun tidak lengkap)
                csv_logger.log_data(current_time, gps_latlon_data, GDT, pos_estimated, smoothed_position)
                
                print(f"GPS: {len(GDT)}/3 (tidak lengkap) | Posisi: [{pos_estimated[0]:.3f}, {pos_estimated[1]:.3f}]")
            else:
                print(f"Tidak ada data GPS valid dari semua sensor")
                
        except Exception as e:
            print(f"Error proses: {e}")
        
        time.sleep(0.1)  # delay 100ms, sesuaikan dengan rate GPS

except KeyboardInterrupt:
    # Hentikan semua thread saat program dihentikan
    print("\nMenghentikan program...")
    for gps in gps_readers:
        gps.stop()
    
    csv_logger.close()
    print(f"Data tersimpan di: {csv_logger.filename}")
