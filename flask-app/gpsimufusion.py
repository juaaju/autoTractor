import numpy as np
import math
import time
import os
import csv
from typing import Tuple, Optional
from queue import Queue
from threading import Thread, Lock
from dataclasses import dataclass
from datetime import datetime
from mpu9250read import mpu9250
from gpsread import GPSReader

# Definisikan konstanta sampling rate
SAMPLING_RATE = 100  # Hz

# Definisikan kelas SensorData
@dataclass
class SensorData:
    timestamp: float
    imu_accel_x: float
    imu_accel_y: float
    imu_gyro_z: float
    gps_lat: Optional[float]
    gps_lng: Optional[float]

class EKFLocalization:
    """
    Kelas untuk fusi sensor antara odometri IMU dan GPS menggunakan Extended Kalman Filter
    """
    def __init__(self):
        # State vector: [x, y, heading, vx, vy, omega]
        self.state = np.zeros(6)
        
        # Matriks kovarians - menggambarkan ketidakpastian state
        # Nilai awal diset cukup tinggi untuk posisi dan orientasi karena belum ada data
        self.P = np.diag([100.0, 100.0, 10.0, 1.0, 1.0, 1.0])
        
        # Noise proses - menggambarkan ketidakpastian model
        # Nilai lebih tinggi berarti kurang percaya pada model prediksi
        self.Q = np.diag([0.1, 0.1, 0.05, 0.5, 0.5, 0.3])
        
        # Noise pengukuran GPS - menggambarkan ketidakpastian GPS
        # Nilai lebih tinggi berarti kurang percaya pada data GPS
        self.R_gps = np.diag([5.0, 5.0])  # Asumsi akurasi GPS sekitar 5 meter
        
        # Faktor konversi GPS ke meter (tergantung lokasi)
        # Nilai untuk sekitar ekuator, perlu disesuaikan dengan lokasi Anda
        self.lat_to_m = 111320.0  # 1 derajat latitude ≈ 111.32 km
        self.lng_to_m = 110540.0  # 1 derajat longitude ≈ 110.54 km (di ekuator)
        
        # Origin GPS (titik awal) - akan diinisialisasi dengan data GPS pertama
        self.origin_lat = None
        self.origin_lng = None
        
        # Timestamp terakhir
        self.last_timestamp = None
        
    def initialize_origin(self, lat, lng):
        """Inisialisasi titik asal koordinat GPS"""
        self.origin_lat = lat
        self.origin_lng = lng
        
    def gps_to_local(self, lat, lng) -> Tuple[float, float]:
        """Konversi koordinat GPS ke koordinat lokal dalam meter"""
        if self.origin_lat is None or self.origin_lng is None:
            self.initialize_origin(lat, lng)
            return 0.0, 0.0
            
        # Konversi delta koordinat ke meter
        x = (lng - self.origin_lng) * self.lng_to_m
        y = (lat - self.origin_lat) * self.lat_to_m
        
        return x, y
        
    def predict(self, imu_data, dt):
        """
        Langkah prediksi EKF menggunakan data IMU dan odometri
        
        Args:
            imu_data: Data dari sensor IMU dan odometri
            dt: Delta waktu sejak update terakhir
        """
        if dt <= 0:
            return
            
        # Model gerak sederhana berdasarkan kecepatan saat ini
        # x += vx * dt
        # y += vy * dt
        # theta += omega * dt
        
        # State vector: [x, y, heading, vx, vy, omega]
        x, y, theta, vx, vy, omega = self.state
        
        # Prediksi state baru
        self.state[0] += vx * dt
        self.state[1] += vy * dt
        self.state[2] += omega * dt
        
        # Update kecepatan dari data IMU
        self.state[3] = imu_data.vx  # kecepatan x dari odometri IMU
        self.state[4] = imu_data.vy  # kecepatan y dari odometri IMU
        self.state[5] = imu_data.omega  # kecepatan sudut dari odometri IMU
        
        # Normalisasi sudut
        self.state[2] = self.state[2] % (2 * math.pi)
        
        # Untuk EKF, kita perlu menghitung Jacobian dari model gerak
        # Untuk model sederhana ini, Jacobian adalah:
        F = np.eye(6)
        F[0, 3] = dt  # ∂x/∂vx
        F[1, 4] = dt  # ∂y/∂vy
        F[2, 5] = dt  # ∂theta/∂omega
        
        # Update matriks kovarians
        self.P = F @ self.P @ F.T + self.Q
        
    def correct_with_gps(self, gps_lat, gps_lng):
        """
        Langkah koreksi EKF menggunakan data GPS
        
        Args:
            gps_lat: Latitude dari GPS
            gps_lng: Longitude dari GPS
        """
        if gps_lat is None or gps_lng is None:
            return
        
        # Konversi GPS ke koordinat lokal
        gps_x, gps_y = self.gps_to_local(gps_lat, gps_lng)
        
        # Matriks pengukuran - menghubungkan state dengan pengukuran
        # Untuk GPS, kita hanya mengukur posisi x dan y
        H = np.zeros((2, 6))
        H[0, 0] = 1.0  # Pengukuran x
        H[1, 1] = 1.0  # Pengukuran y
        
        # Selisih antara pengukuran dan prediksi
        z = np.array([gps_x, gps_y])
        y = z - self.state[0:2]
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update kovarians
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        
    def update(self, odometry_data, gps_data, timestamp):
        """
        Update lengkap EKF dengan data odometri dan GPS
        
        Args:
            odometry_data: Data odometri IMU (x, y, heading, vx, vy, omega)
            gps_data: Data GPS (lat, lng)
            timestamp: Timestamp saat ini
            
        Returns:
            Tuple[float, float, float]: Estimasi posisi x, y, dan heading
        """
        # Pada update pertama, inisialisasi
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            
            # Inisialisasi state jika ada data odometri
            if odometry_data is not None:
                self.state[0] = odometry_data[0]  # x
                self.state[1] = odometry_data[1]  # y
                self.state[2] = odometry_data[2]  # heading
                
            # Inisialisasi origin GPS jika ada data GPS
            if gps_data is not None and gps_data[0] is not None:
                self.initialize_origin(gps_data[0], gps_data[1])
                
            return self.state[0], self.state[1], self.state[2]
        
        # Hitung delta waktu
        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        
        # Persiapkan data IMU untuk prediksi
        imu_data_struct = type('IMUData', (), {
            'vx': odometry_data[3] if odometry_data is not None else 0.0,
            'vy': odometry_data[4] if odometry_data is not None else 0.0,
            'omega': odometry_data[5] if odometry_data is not None else 0.0
        })
        
        # Langkah prediksi
        self.predict(imu_data_struct, dt)
        
        # Langkah koreksi dengan GPS (jika tersedia)
        if gps_data is not None and gps_data[0] is not None:
            self.correct_with_gps(gps_data[0], gps_data[1])
        
        # Kembalikan estimasi posisi dan orientasi terkini
        return self.state[0], self.state[1], self.state[2]

# Kelas untuk odometry IMU
class IMUOdometry:
    def __init__(self):
        # Posisi awal (0,0) dan heading awal 0 derajat (menghadap ke sumbu x positif)
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # dalam radian
        
        # Kecepatan
        self.vx = 0.0
        self.vy = 0.0
        
        # Timestamp terakhir untuk kalkulasi delta time
        self.last_timestamp = None
        
        # Faktor koreksi - bisa disesuaikan berdasarkan pengujian
        self.accel_factor = 1.0  # faktor untuk mengurangi noise accelerometer
        self.gyro_factor = 1.0   # faktor untuk mengurangi drift gyroscope
        
        # Threshold untuk mendeteksi gerakan (mengurangi drift saat diam)
        self.accel_threshold = 0.05  # m/s² (percepatan minimum yang dideteksi sebagai gerakan)
    
    def update(self, sensor_data: SensorData):
        """
        Update posisi dan orientasi berdasarkan data sensor IMU
        
        Args:
            sensor_data: Data sensor dari IMU
        
        Returns:
            Tuple[float, float, float]: Posisi x, y dan heading saat ini
        """
        # Pada update pertama, inisialisasi timestamp
        if self.last_timestamp is None:
            self.last_timestamp = sensor_data.timestamp
            return self.x, self.y, self.heading
        
        # Hitung delta waktu
        dt = sensor_data.timestamp - self.last_timestamp
        self.last_timestamp = sensor_data.timestamp
        
        # Pastikan dt tidak terlalu besar (penanganan jika ada delay)
        if dt > 0.1:  # Jika delay lebih dari 0.1 detik, batasi untuk menghindari error besar
            dt = 0.1
        
        # Update heading berdasarkan gyro Z
        # Konversi dari deg/s ke rad/s (jika belum dalam rad/s)
        gyro_z_rad = math.radians(sensor_data.imu_gyro_z) * self.gyro_factor
        self.heading += gyro_z_rad * dt
        
        # Normalisasi heading ke dalam rentang [0, 2π]
        self.heading = self.heading % (2 * math.pi)
        
        # Terapkan percepatan dalam koordinat global
        # Rotasi vektor percepatan sesuai dengan heading saat ini
        accel_magnitude = math.sqrt(sensor_data.imu_accel_x**2 + sensor_data.imu_accel_y**2)
        
        # Hanya proses percepatan jika di atas threshold (mengurangi drift saat diam)
        if accel_magnitude > self.accel_threshold:
            # Accel X (maju/mundur) dalam koordinat IMU
            accel_x_global = (sensor_data.imu_accel_x * math.cos(self.heading) - 
                              sensor_data.imu_accel_y * math.sin(self.heading)) * self.accel_factor
            
            # Accel Y (kiri/kanan) dalam koordinat IMU
            accel_y_global = (sensor_data.imu_accel_x * math.sin(self.heading) + 
                              sensor_data.imu_accel_y * math.cos(self.heading)) * self.accel_factor
            
            # Update kecepatan
            self.vx += accel_x_global * dt
            self.vy += accel_y_global * dt
            
            # Penerapan faktor redaman untuk mengurangi drift
            damping_factor = 0.98
            self.vx *= damping_factor
            self.vy *= damping_factor
            
            # Update posisi
            self.x += self.vx * dt
            self.y += self.vy * dt
        else:
            # Jika percepatan di bawah threshold, anggap tidak bergerak
            # Redaman lebih besar ketika diam untuk mencegah drift
            self.vx *= 0.8
            self.vy *= 0.8
        
        return self.x, self.y, self.heading
    
    def reset(self):
        """Reset posisi dan orientasi ke nilai awal"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_timestamp = None
    
    def set_position(self, x, y, heading=None):
        """
        Set posisi secara manual (misalnya dari GPS untuk koreksi)
        
        Args:
            x: Posisi x baru
            y: Posisi y baru
            heading: Heading baru (opsional)
        """
        self.x = x
        self.y = y
        if heading is not None:
            self.heading = heading

class DataCollector:
    def __init__(self, log_to_csv=True):
        self.data_queue = Queue()
        self.last_gps_data = (None, None)  # (lat, lng)
        self.running = True
        self.lock = Lock()
        self.log_to_csv = log_to_csv

        self.sampling_interval = 1.0 / SAMPLING_RATE
        
        # Initialize IMU odometry
        self.imu_odometry = IMUOdometry()
        self.current_position = (0.0, 0.0, 0.0)  # x, y, heading
        
        # Initialize EKF
        self.ekf = EKFLocalization()
        self.ekf_position = (0.0, 0.0, 0.0)  # x, y, heading dari EKF
        
        # Setup CSV logging
        if self.log_to_csv:
            # Buat direktori log jika belum ada
            if not os.path.exists('logs'):
                os.makedirs('logs')
                
            # Buat nama file berdasarkan waktu
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_filename = f"logs/sensor_data_{timestamp}.csv"
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Tulis header
            self.csv_writer.writerow([
                'timestamp', 
                'imu_accel_x', 'imu_accel_y', 'imu_gyro_z',
                'gps_lat', 'gps_lng',
                'odom_x', 'odom_y', 'odom_heading', 'odom_vx', 'odom_vy',
                'ekf_x', 'ekf_y', 'ekf_heading'
            ])
            
            print(f"Log data akan disimpan di: {self.csv_filename}")
        
        try:
            # Initialize sensors
            self.mpu = mpu9250(0x68)
            self.gps = GPSReader(port="/dev/ttyTHS0")
            
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
        except Exception as e:
            print(f"Error initializing sensors: {e}")
            if self.log_to_csv:
                self.csv_file.close()
            raise

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
        
        if len(accel_x_samples) == 0:
            print("Tidak ada sampel yang berhasil diambil untuk kalibrasi")
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
                
                # Update odometry IMU
                x, y, heading = self.imu_odometry.update(sensor_data)
                self.current_position = (x, y, heading)
                
                # Update EKF dengan data odometri dan GPS
                vx, vy = self.imu_odometry.vx, self.imu_odometry.vy
                omega = math.radians(gyro_z_calibrated)  # dalam rad/s
                
                x_ekf, y_ekf, heading_ekf = self.ekf.update(
                    odometry_data=(x, y, heading, vx, vy, omega),
                    gps_data=(gps_lat, gps_lng),
                    timestamp=current_time
                )
                
                # Simpan hasil EKF
                self.ekf_position = (x_ekf, y_ekf, heading_ekf)
                
                # Log ke CSV jika diperlukan
                if self.log_to_csv:
                    self._log_to_csv(
                        sensor_data, 
                        (x, y, heading, vx, vy), 
                        (x_ekf, y_ekf, heading_ekf)
                    )
                
                self.data_queue.put(sensor_data)
                
            except Exception as e:
                print(f"IMU reading error: {e}")
            
            time.sleep(self.sampling_interval)

    def _log_to_csv(self, sensor_data, odom_data, ekf_data):
        """
        Log data ke file CSV
        
        Args:
            sensor_data: Data sensor mentah
            odom_data: Data odometri (x, y, heading, vx, vy)
            ekf_data: Data EKF (x, y, heading)
        """
        try:
            timestamp_str = datetime.fromtimestamp(sensor_data.timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")
            
            self.csv_writer.writerow([
                timestamp_str,
                sensor_data.imu_accel_x, sensor_data.imu_accel_y, sensor_data.imu_gyro_z,
                sensor_data.gps_lat, sensor_data.gps_lng,
                odom_data[0], odom_data[1], odom_data[2], odom_data[3], odom_data[4],
                ekf_data[0], ekf_data[1], ekf_data[2]
            ])
            
            # Flush untuk menulis ke disk secara langsung
            self.csv_file.flush()
            
        except Exception as e:
            print(f"Error writing to CSV: {e}")

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
    
    def get_current_position(self) -> Tuple[float, float, float]:
        """
        Mendapatkan posisi terkini dari odometri IMU
        
        Returns:
            Tuple[float, float, float]: Posisi x (meter), y (meter), dan heading (radian)
        """
        return self.current_position
    
    def get_ekf_position(self) -> Tuple[float, float, float]:
        """
        Mendapatkan estimasi posisi dari EKF
        
        Returns:
            Tuple[float, float, float]: Posisi x, y, dan heading dari EKF
        """
        return self.ekf_position
    
    def reset_odometry(self):
        """Reset odometri IMU ke posisi awal (0,0,0)"""
        self.imu_odometry.reset()
        self.current_position = (0.0, 0.0, 0.0)
    
    def set_odometry_position(self, x, y, heading=None):
        """
        Set posisi odometri secara manual
        
        Args:
            x: Posisi x baru (meter)
            y: Posisi y baru (meter)
            heading: Heading baru dalam radian (opsional)
        """
        self.imu_odometry.set_position(x, y, heading)
        if heading is None:
            self.current_position = (x, y, self.current_position[2])
        else:
            self.current_position = (x, y, heading)

    def stop(self):
        self.running = False
        self.gps_thread.join()
        self.imu_thread.join()
        self.gps.close()  # Menutup koneksi GPS dengan benar
        
        if self.log_to_csv:
            self.csv_file.close()
            print(f"Log data tersimpan di: {self.csv_filename}")

if __name__ == "__main__":
    try:
        log_to_csv = True  # Set ke False jika tidak ingin menyimpan ke CSV
        collector = DataCollector(log_to_csv=log_to_csv)
        print("Sistem aktif. Mengumpulkan data...")
        
        # Fungsi untuk membersihkan terminal
        def clear_terminal():
            # Untuk sistem Unix/Linux/MacOS
            if os.name == 'posix':
                os.system('clear')
            # Untuk sistem Windows
            else:
                os.system('cls')
        
        # Inisialisasi waktu mulai untuk menghitung durasi
        start_time = time.time()
        
        while True:
            data = collector.get_latest_data()
            x, y, heading = collector.get_current_position()
            x_ekf, y_ekf, heading_ekf = collector.get_ekf_position()
            
            heading_deg = math.degrees(heading) % 360  # Konversi ke derajat untuk tampilan
            heading_ekf_deg = math.degrees(heading_ekf) % 360  # Konversi ke derajat untuk tampilan
            
            # Hitung waktu berjalan
            elapsed_time = time.time() - start_time
            hours, remainder = divmod(elapsed_time, 3600)
            minutes, seconds = divmod(remainder, 60)
            
            # Bersihkan terminal dan perbarui tampilan
            clear_terminal()
            
            print("=" * 50)
            print(f"DATA SENSOR - {datetime.fromtimestamp(data.timestamp).strftime('%H:%M:%S.%f')}")
            print(f"Durasi Logging: {int(hours):02d}:{int(minutes):02d}:{int(seconds):02d}")
            print("=" * 50)
            print(f"GPS      : Lat={data.gps_lat}, Lng={data.gps_lng}")
            print(f"IMU      : AccX={data.imu_accel_x:.4f} m/s², AccY={data.imu_accel_y:.4f} m/s², GyroZ={data.imu_gyro_z:.4f} deg/s")
            print(f"ODOMETRI : X={x:.2f}m, Y={y:.2f}m, Heading={heading_deg:.1f}°")
            print(f"EKF      : X={x_ekf:.2f}m, Y={y_ekf:.2f}m, Heading={heading_ekf_deg:.1f}°")
            if log_to_csv:
                print(f"Log File : {collector.csv_filename}")
            print("=" * 50)
            print("Tekan Ctrl+C untuk berhenti dan menyimpan data")
            
            time.sleep(0.5)  # Update tampilan setiap 0.5 detik
            
    except KeyboardInterrupt:
        print("\nMenghentikan sistem...")
        collector.stop()
        print("Sistem berhenti.")
    except Exception as e:
        print(f"Error: {e}")
        if 'collector' in locals():
            collector.stop()