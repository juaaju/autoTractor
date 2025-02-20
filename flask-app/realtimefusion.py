import numpy as np
import math
import time
from typing import Tuple, Optional
from queue import Queue
from threading import Thread, Lock
from dataclasses import dataclass
from datetime import datetime
from mpuread import mpu6050
from gpsread import GPSReader
from ekfnparam import EKFSensorFusion

@dataclass
class SensorData:
    timestamp: float
    imu_accel_x: float
    imu_accel_y: float
    imu_gyro_z: float
    gps_lat: Optional[float] = None
    gps_lng: Optional[float] = None

class DataCollector:
    def __init__(self):
        self.data_queue = Queue()
        self.last_gps_data = (None, None)  # (lat, lng)
        self.running = True
        self.lock = Lock()
        
        # Initialize sensors
        self.mpu = mpu6050(0x68)
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

    def calibrate_imu(self, samples=500, delay=0.01):
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
            
            time.sleep(0.004)  # 250Hz IMU update rate

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
        
        # Initialize logging
        self.log_file = open(f"sensor_fusion_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
        self.log_file.write("timestamp,gps_lat,gps_lng,imu_accel_x,imu_accel_y,imu_gyro_z,"
                           "estimated_x,estimated_y,estimated_heading\n")

    def process_sensor_data(self, sensor_data: SensorData):
        # Prediction step
        self.ekf.predict()

        # Convert GPS coordinates to local coordinates if GPS data exists
        if sensor_data.gps_lat is not None and sensor_data.gps_lng is not None:
            # You'll need to implement this conversion based on your reference point
            x, y = self.gps_to_local_coordinates(sensor_data.gps_lat, sensor_data.gps_lng)
        else:
            # Use last known position if GPS is unavailable
            x, y = self.ekf.state[0:2]

        # Update step with measurements
        measurements = np.array([
            x,
            y,
            sensor_data.imu_accel_x,  # You might need to transform these based on your setup
            sensor_data.imu_gyro_z
        ])
        
        self.ekf.update(measurements)

        # Log data
        estimated_state = self.ekf.state
        self.log_file.write(f"{sensor_data.timestamp},{sensor_data.gps_lat},{sensor_data.gps_lng},"
                           f"{sensor_data.imu_accel_x},{sensor_data.imu_accel_y},{sensor_data.imu_gyro_z},"
                           f"{estimated_state[0]},{estimated_state[1]},{estimated_state[2]}\n")
        
        return estimated_state

    def gps_to_local_coordinates(self, lat, lng, REF_LAT=0, REF_LNG=0):
        # Implement GPS to local coordinate conversion
        # This will depend on your reference point and coordinate system
        # Example (you'll need to modify this based on your needs):
        EARTH_RADIUS = 6371000  # meters
        #REF_LAT Your reference latitude
        #REF_LNG Your reference longitude
        
        x = EARTH_RADIUS * math.cos(REF_LAT) * (lng - REF_LNG)
        y = EARTH_RADIUS * (lat - REF_LAT)
        
        return x, y

    def close(self):
        self.log_file.close()

def main():
    # Initialize data collector
    collector = DataCollector()
    
    # Initialize EKF with 250Hz update rate
    dt = 1/250.0
    fusion = RealTimeEKFSensorFusion(dt)

    try:
        print("Starting sensor fusion...")
        while True:
            # Get latest sensor data
            sensor_data = collector.get_latest_data()
            
            # Process data through EKF
            estimated_state = fusion.process_sensor_data(sensor_data)
            
            # Print current state
            print(f"Position: ({estimated_state[0]:.2f}, {estimated_state[1]:.2f}), "
                  f"Heading: {math.degrees(estimated_state[2]):.1f}°")

    except KeyboardInterrupt:
        print("\nStopping sensor fusion...")
    finally:
        collector.stop()
        fusion.close()

#data yang dikirim itu gps raw, imu odometry raw, dan hasil EKF, kirim untuk ditampilkan ke maps


if __name__ == "__main__":
    main()

