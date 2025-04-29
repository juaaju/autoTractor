import numpy as np
import math
import time
from typing import Tuple, Optional
from queue import Queue
from threading import Thread, Lock
from dataclasses import dataclass
from datetime import datetime
from mpu9250read import mpu9250
from gpsread import GPSReader
from ekfnparam import EKFSensorFusion

SAMPLING_RATE = 250.0  # Hz

@dataclass
class SensorData:
    timestamp: float
    imu_accel_x: float
    imu_accel_y: float
    imu_gyro_z: float
    gps_lat: Optional[float] = None
    gps_lng: Optional[float] = None

class ComplementaryFilter:
    """
    Implementasi Filter Komplementer untuk mengurangi drift saat GPS tidak tersedia
    """
    def __init__(self, alpha=0.98):
        """
        Inisialisasi filter komplementer
        Args:
            alpha: Bobot untuk data gyroscope (nilai lebih tinggi = lebih mempercayai gyro)
        """
        self.alpha = alpha
        self.last_timestamp = None
        self.heading = 0.0  # dalam radian
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_gps_position = (0.0, 0.0)
        self.gps_available_before = False
        
    def update(self, sensor_data: SensorData, estimated_state=None):
        """
        Update filter dengan data sensor baru
        
        Args:
            sensor_data: Data sensor terbaru
            estimated_state: Hasil estimasi EKF jika tersedia (x, y, heading)
        
        Returns:
            Tuple posisi dan heading (x, y, heading)
        """
        current_time = sensor_data.timestamp
        
        # Inisialisasi timestamp jika pertama kali
        if self.last_timestamp is None:
            self.last_timestamp = current_time
            if estimated_state is not None:
                self.position_x, self.position_y, self.heading = estimated_state
            return (self.position_x, self.position_y, self.heading)
        
        # Hitung delta waktu
        dt = current_time - self.last_timestamp
        if dt <= 0:
            return (self.position_x, self.position_y, self.heading)
        
        # Update heading dari gyro (integrasi)
        gyro_heading_change = sensor_data.imu_gyro_z * dt  # dalam radian
        gyro_heading = self.heading + gyro_heading_change
        
        # Estimasi heading dari accelerometer (untuk koreksi drift)
        accel_heading = math.atan2(sensor_data.imu_accel_y, sensor_data.imu_accel_x)
        
        # Mode update tergantung ketersediaan GPS
        if sensor_data.gps_lat is not None and sensor_data.gps_lng is not None:
            # GPS tersedia - gunakan EKF
            if estimated_state is not None:
                # Ambil hasil EKF
                self.position_x, self.position_y, self.heading = estimated_state
                
                # Reset velocity dengan data GPS baru
                if self.gps_available_before:
                    dx = self.position_x - self.last_gps_position[0]
                    dy = self.position_y - self.last_gps_position[1]
                    self.velocity_x = dx / dt
                    self.velocity_y = dy / dt
                
                self.last_gps_position = (self.position_x, self.position_y)
                self.gps_available_before = True
            
        else:
            # GPS tidak tersedia - gunakan filter komplementer untuk meminimalkan drift
            
            # Gabungkan heading dari gyro dan accelerometer
            self.heading = self.alpha * gyro_heading + (1 - self.alpha) * accel_heading
            
            # Update posisi berdasarkan velocity dan heading
            self.position_x += self.velocity_x * dt
            self.position_y += self.velocity_y * dt
            
            # Aplikasikan koreksi kecil untuk mengurangi drift velocity dari waktu ke waktu
            # Faktor decay untuk velocity saat tidak ada GPS
            velocity_decay = 0.99  # Kurangi velocity perlahan jika tidak ada pembaruan GPS
            self.velocity_x *= velocity_decay
            self.velocity_y *= velocity_decay
            
            # Jika sebelumnya ada GPS tetapi sekarang tidak, mulai perhitungan dead reckoning
            # dari posisi GPS terakhir yang diketahui
            self.gps_available_before = False
            
        self.last_timestamp = current_time
        return (self.position_x, self.position_y, self.heading)

class DataCollector:
    def __init__(self):
        self.data_queue = Queue()
        self.last_gps_data = (None, None)  # (lat, lng)
        self.running = True
        self.lock = Lock()

        self.sampling_interval = 1.0 / SAMPLING_RATE
        
        # Initialize sensors
        self.mpu = mpu9250(0x68)
        self.gps = GPSReader(port="/dev/ttyACM0")
        
        # Calibration offsets
        self.accel_offset_x = 0
        self.accel_offset_y = 0
        self.gyro_offset_z = 0
        
        # Perform initial calibration
        print("Memulai kalibrasi IMU...")
        print("Pastikan sensor dalam keadaan diam...")
        self.calibrate_imu()
        print("Kalibrasi selesai!")
        print(f"Offset Accelerometer X: {self.accel_offset_x:.4f} m/s²")
        print(f"Offset Accelerometer Y: {self.accel_offset_y:.4f} m/s²")
        print(f"Offset Gyroscope Z: {self.gyro_offset_z:.4f} deg/s")
        
        # Start GPS and IMU threads
        self.gps_thread = Thread(target=self._collect_gps_data)
        self.imu_thread = Thread(target=self._collect_imu_data)
        self.gps_thread.daemon = True
        self.imu_thread.daemon = True
        self.gps_thread.start()
        self.imu_thread.start()

    def calibrate_imu(self, samples=50, delay=0.04):
        """
        Kalibrasi IMU dengan mengambil beberapa sampel saat diam
        Args:
            samples: Jumlah sampel untuk kalibrasi
            delay: Delay antar sampel (dalam detik)
        """
        accel_x_samples = []
        accel_y_samples = []
        gyro_z_samples = []
        
        print(f"Mengambil {samples} sampel...")
        for _ in range(samples):
            try:
                accel_data = self.mpu.get_accel_data()
                gyro_data = self.mpu.get_gyro_data()
                
                accel_x_samples.append(accel_data['x'])
                accel_y_samples.append(accel_data['y'])
                gyro_z_samples.append(gyro_data['z'])
                
                time.sleep(delay)
                
            except Exception as e:
                print(f"Error during calibration: {e}")
                return
        
        # Hitung rata-rata untuk mendapatkan offset
        self.accel_offset_x = np.mean(accel_x_samples)
        self.accel_offset_y = np.mean(accel_y_samples)
        self.gyro_offset_z = np.mean(gyro_z_samples)
        
        # Hitung standar deviasi untuk melihat noise
        accel_x_std = np.std(accel_x_samples)
        accel_y_std = np.std(accel_y_samples)
        gyro_z_std = np.std(gyro_z_samples)
        
        print("\nStatistik Noise:")
        print(f"Std Dev Accel X: {accel_x_std:.4f} m/s²")
        print(f"Std Dev Accel Y: {accel_y_std:.4f} m/s²")
        print(f"Std Dev Gyro Z: {gyro_z_std:.4f} deg/s")

    def _collect_imu_data(self):
        while self.running:
            try:
                accel_data = self.mpu.get_accel_data()
                gyro_data = self.mpu.get_gyro_data()
                
                # Kompensasi dengan offset kalibrasi
                accel_x_calibrated = accel_data['x'] - self.accel_offset_x
                accel_y_calibrated = accel_data['y'] - self.accel_offset_y
                gyro_z_calibrated = gyro_data['z'] - self.gyro_offset_z
                
                current_time = time.time()
                
                with self.lock:
                    gps_lat, gps_lng = self.last_gps_data

                sensor_data = SensorData(
                    timestamp=current_time,
                    imu_accel_x=accel_x_calibrated,
                    imu_accel_y=accel_y_calibrated,
                    imu_gyro_z=gyro_z_calibrated,
                    gps_lat=gps_lat,
                    gps_lng=gps_lng
                )
                
                self.data_queue.put(sensor_data)
                
            except Exception as e:
                print(f"IMU reading error: {e}")
            
            # time.sleep(0.004)  # 250Hz IMU update rate
            time.sleep(self.sampling_interval)

    def _collect_gps_data(self):
        while self.running:
            try:
                gps_data = self.gps.read()  # Menggunakan method read() dari GPSReader
                if gps_data is not None:  # Hanya update jika ada data valid
                    with self.lock:
                        self.last_gps_data = gps_data
            except Exception as e:
                print(f"GPS reading error: {e}")
            time.sleep(0.2)  # 5Hz GPS update rate

    def get_latest_data(self) -> SensorData:
        return self.data_queue.get()

    def stop(self):
        self.running = False
        self.gps_thread.join()
        self.imu_thread.join()
        self.gps.close()  # Menutup koneksi GPS dengan benar

class RealTimeEKFSensorFusion:
    def __init__(self, dt: float):
        self.ekf = EKFSensorFusion(dt)  # Your original EKF class
        
        # Tambahkan filter komplementer untuk mengurangi drift
        self.complementary_filter = ComplementaryFilter(alpha=0.98)
        
        # Tambahkan flag untuk melacak ketersediaan GPS
        self.last_gps_timestamp = 0
        self.gps_timeout = 5.0  # 5 detik timeout untuk GPS
        
        # Inisialisasi titik referensi GPS
        self.ref_lat = None
        self.ref_lng = None
        self.is_ref_initialized = False
        self.ref_init_samples = []  # Untuk menyimpan sampel awal GPS
        self.ref_init_time = 30  # Waktu inisialisasi referensi (detik)
        self.first_gps_time = None  # Menyimpan waktu pertama GPS tersedia
        
        # Generate log file name and store the path
        log_filename = f"sensor_fusion_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file_path = log_filename
        
        # Initialize logging
        self.log_file = open(self.log_file_path, "w")
        self.log_file.write("timestamp,gps_lat,gps_lng,imu_accel_x,imu_accel_y,imu_gyro_z,"
                           "estimated_x,estimated_y,estimated_heading,filter_mode,ref_lat,ref_lng\n")

    def process_sensor_data(self, sensor_data: SensorData):
        # Prediction step
        self.ekf.predict()
        # With this code block:
        current_time = sensor_data.timestamp
        if hasattr(self, 'last_process_time'):
            actual_dt = current_time - self.last_process_time
            # Use actual_dt for EKF prediction
            self.ekf.dt = actual_dt  # Update EKF's dt dynamically
            self.ekf.predict()
        else:
            # First iteration
            self.ekf.predict()  # Use default dt for first prediction
        self.last_process_time = current_time
        # Check if GPS is available
        gps_available = sensor_data.gps_lat is not None and sensor_data.gps_lng is not None
        
        if gps_available:
            self.last_gps_timestamp = sensor_data.timestamp
            
            # Convert GPS coordinates to local coordinates
            x, y = self.gps_to_local_coordinates(sensor_data.gps_lat, sensor_data.gps_lng)
            
            # Hanya lakukan update EKF jika titik referensi telah diinisialisasi
            # atau jika ini adalah data GPS awal untuk inisialisasi
            if self.is_ref_initialized:
                # Update step with measurements
                measurements = np.array([
                    x,
                    y,
                    sensor_data.imu_accel_x,
                    sensor_data.imu_gyro_z
                ])
                
                self.ekf.update(measurements)
                estimated_state = self.ekf.state
                filter_mode = "EKF"
            else:
                # Dalam fase inisialisasi, gunakan nilai dari GPS langsung
                # tanpa memperbarui EKF
                estimated_state = self.ekf.state
                estimated_state[0] = x
                estimated_state[1] = y
                filter_mode = "Initializing"
            
        else:
            # Check if GPS has been unavailable for too long
            gps_timeout_occurred = (sensor_data.timestamp - self.last_gps_timestamp) > self.gps_timeout
            
            if gps_timeout_occurred:
                # Use complementary filter to minimize drift
                x, y = self.ekf.state[0:2]  # Get last estimated position
                
                # Update only with IMU measurements
                estimated_state = self.ekf.state  # Get current state for complementary filter
                
                # Update complementary filter and get corrected state
                comp_x, comp_y, comp_heading = self.complementary_filter.update(
                    sensor_data, 
                    estimated_state=[estimated_state[0], estimated_state[1], estimated_state[2]]
                )
                
                # Update EKF state with complementary filter output
                estimated_state[0] = comp_x
                estimated_state[1] = comp_y
                estimated_state[2] = comp_heading
                
                filter_mode = "Complementary"
                
            else:
                # Keep using EKF for short GPS outages
                x, y = self.ekf.state[0:2]
                
                # Update step with last known position and IMU
                measurements = np.array([
                    x,
                    y,
                    sensor_data.imu_accel_x,
                    sensor_data.imu_gyro_z
                ])
                
                self.ekf.update(measurements)
                estimated_state = self.ekf.state
                filter_mode = "EKF-DR"  # EKF with Dead Reckoning

        # Log data
        self.log_file.write(f"{sensor_data.timestamp},{sensor_data.gps_lat},{sensor_data.gps_lng},"
                           f"{sensor_data.imu_accel_x},{sensor_data.imu_accel_y},{sensor_data.imu_gyro_z},"
                           f"{estimated_state[0]},{estimated_state[1]},{estimated_state[2]},{filter_mode},"
                           f"{self.ref_lat if self.is_ref_initialized else 'None'},{self.ref_lng if self.is_ref_initialized else 'None'}\n")
        
        # Flush to ensure data is written to disk immediately
        self.log_file.flush()
        
        return estimated_state, filter_mode

    def gps_to_local_coordinates(self, lat, lng):
        """
        Mengkonversi koordinat GPS (lat, lng) ke koordinat kartesian lokal (x, y)
        menggunakan titik referensi yang diinisialisasi dari posisi awal
        """
        # Konstanta
        EARTH_RADIUS = 6371000  # meters
        
        if lat is None or lng is None:
            # Return default jika GPS tidak tersedia
            return 0.0, 0.0
        
        # Inisialisasi referensi jika belum
        if not self.is_ref_initialized:
            # Jika ini adalah sampel GPS pertama
            if self.first_gps_time is None:
                self.first_gps_time = time.time()
                print(f"GPS pertama terdeteksi. Mengumpulkan sampel untuk {self.ref_init_time} detik...")
            
            # Tambahkan sampel ke array inisialisasi
            self.ref_init_samples.append((lat, lng))
            
            # Cek apakah sudah cukup waktu untuk menentukan referensi
            if time.time() - self.first_gps_time >= self.ref_init_time:
                # Hitung rata-rata dari sampel yang dikumpulkan
                if len(self.ref_init_samples) > 0:
                    lats, lngs = zip(*self.ref_init_samples)
                    self.ref_lat = np.mean(lats)
                    self.ref_lng = np.mean(lngs)
                    self.is_ref_initialized = True
                    print(f"Titik referensi GPS diinisialisasi: ({self.ref_lat:.6f}, {self.ref_lng:.6f})")
                else:
                    # Gunakan nilai saat ini jika tidak ada sampel yang dikumpulkan
                    self.ref_lat = lat
                    self.ref_lng = lng
                    self.is_ref_initialized = True
                    print(f"Titik referensi GPS diinisialisasi dengan sampel tunggal: ({self.ref_lat:.6f}, {self.ref_lng:.6f})")
        
        # Gunakan referensi untuk konversi atau nilai saat ini jika belum terinisialisasi
        ref_lat = self.ref_lat if self.is_ref_initialized else lat
        ref_lng = self.ref_lng if self.is_ref_initialized else lng
        
        # Konversi ke koordinat lokal menggunakan proyeksi Equirectangular
        # (proyeksi sederhana yang cukup akurat untuk area kecil)
        x = EARTH_RADIUS * math.cos(math.radians(ref_lat)) * math.radians(lng - ref_lng)
        y = EARTH_RADIUS * math.radians(lat - ref_lat)
        
        return x, y

    def close(self):
        self.log_file.close()

def main():
    # Initialize data collector
    collector = DataCollector()
    dt = 1.0 / SAMPLING_RATE
    # Initialize EKF with 250Hz update rate
    # dt = 1/250.0
    fusion = RealTimeEKFSensorFusion(dt)
    raw_heading = 0.0

    try:
        print("Starting sensor fusion...")
        print("Mode filter akan beralih otomatis antara EKF dan Complementary filter")
        
        # Untuk statistik
        gps_outage_count = 0
        last_mode = None
        
        while True:
            # Get latest sensor data
            sensor_data = collector.get_latest_data()

            gyro_z_rad = math.radians(sensor_data.imu_gyro_z)
            
            raw_heading += gyro_z_rad * dt
            
            # Normalize heading to [0, 2π]
            raw_heading = raw_heading % (2 * math.pi)
            if raw_heading < 0:
                raw_heading += 2 * math.pi
                
            print("Raw Heading (rad): ", raw_heading)
            print("Raw Heading (deg): ", math.degrees(raw_heading))
            
            # Process data through EKF and Complementary filter
            estimated_state, filter_mode = fusion.process_sensor_data(sensor_data)
            
            # Track mode changes
            if last_mode != filter_mode:
                if filter_mode == "Complementary":
                    gps_outage_count += 1
                    print(f"\nPeralihan ke filter komplementer (GPS tidak tersedia) [{gps_outage_count}]")
                elif filter_mode == "EKF" and last_mode == "Complementary":
                    print(f"\nKembali ke EKF (GPS tersedia kembali)")
                last_mode = filter_mode
            
            # Print current state
            print(f"Position: ({estimated_state[0]:.2f}, {estimated_state[1]:.2f}), "
                  f"Heading: {math.degrees(estimated_state[2]):.1f}°, "
                  f"Mode: {filter_mode}", )#end="\r")

    except KeyboardInterrupt:
        print("\n\nStopping sensor fusion...")
        print(f"Total GPS outages: {gps_outage_count}")
    finally:
        collector.stop()
        fusion.close()

#data yang dikirim itu gps raw, imu odometry raw, dan hasil EKF, kirim untuk ditampilkan ke maps


if __name__ == "__main__":
    main()