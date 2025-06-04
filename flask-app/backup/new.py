import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread
from queue import Queue
from gpsread import GPSReader
from dfrobotimu import IMU_WT61PCTTL
import pymap3d as pm
from ekfnparam import EKFSensorFusion  # sesuaikan path modul EKF-mu
from mpu9250read import mpu9250
from mpu6050read import mpu6050

class GPSHandler:
    def __init__(self, port="/dev/ttyUSB0"):
        self.reader = GPSReader(port)
        self.latest_coords = None
        self.running = True
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            coords = self.reader.read()
            if coords:  # coords is a tuple (lat, lon) or None
                lat, lon = coords
                # Convert tuple to dictionary format for consistency
                self.latest_coords = {
                    'latitude': lat,
                    'longitude': lon,
                    'altitude': 0  # default altitude since GPS module doesn't provide it
                }
            # If coords is None, keep the previous value or None
    
    def get_coords(self):
        return self.latest_coords
    
    def stop(self):
        self.running = False
        self.thread.join()
        self.reader.close()

class IMUHandler:
    def __init__(self, i2c=0x68):
        self.imu = mpu9250(i2c)
        self.imu.calibrate()
        self.accel = 0
        self.gyro = 0
        self.running = True
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            data = self.imu.get_all_data()
            self.accel = data['accel']['x']
            self.gyro = math.radians(data['gyro']['z'])  # konversi langsung
    
    def get_data(self):
        return self.accel, self.gyro
    
    def stop(self):
        self.running = False
        self.thread.join()

class IMUHandler:
    def __init__(self, i2c=0x68):
        self.imu = mpu9250(i2c)
        self.imu.calibrate()
        self.accel = 0
        self.gyro = 0
        self.running = True
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            accel_data=self.imu.get_accel_data()
            gyro_data=self.imu.get_gyro_data()
            self.accel = accel_data['accel']['x']
            self.gyro = math.radians(gyro_data['gyro']['z'])  # konversi langsung
    
    def get_data(self):
        return self.accel, self.gyro
    
    def stop(self):
        self.running = False
        self.thread.join()

def simple_latlon2xy(lat, lon, lat0, lon0):
    """
    Konversi koordinat geografis (lat, lon) menjadi koordinat kartesian (x, y) dalam meter
    relatif terhadap titik referensi (lat0, lon0)
    """
    R = 6371000  # Radius bumi dalam meter
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    # x positif ke arah timur
    x = R * dlon * math.cos(math.radians(lat0))
    # y positif ke arah utara
    y = R * dlat
    return x, y

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
    return e, n  # east (x), north (y)

def logging_thread_func(q: Queue, filename: str):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'gps_x', 'gps_y', 'lat', 'lon', 'accel', 'gyro', 
                        'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel'])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def is_gps_valid(gps_coords):
    """
    Validasi GPS coordinates untuk memastikan GPS benar-benar ada sinyal
    """
    if not gps_coords:
        return False
    
    lat = gps_coords.get('latitude', 0)
    lon = gps_coords.get('longitude', 0)
    
    # GPS coordinates 0,0 menandakan tidak ada sinyal valid
    # Juga cek range yang masuk akal untuk koordinat bumi
    if (lat == 0.0 and lon == 0.0) or \
       abs(lat) > 90 or abs(lon) > 180:
        return False
    
    return True

# Tambahkan ini sebelum main() untuk memvalidasi EKF
def debug_ekf_initialization():
    """
    Debug function untuk memastikan EKF berfungsi dengan benar
    """
    dt = 0.1
    ekf = EKFSensorFusion(dt=dt)
    
    print("=== EKF Debug Information ===")
    print(f"Initial state: {ekf.state}")
    print(f"State covariance shape: {ekf.P.shape}")
    print(f"Process noise shape: {ekf.Q.shape}")
    
    # Test predict dengan data IMU dummy
    test_accel = 0.1  # m/s²
    test_gyro = 0.05  # rad/s
    
    initial_state = ekf.state.copy()
    ekf.predict(imu_accel=test_accel, imu_omega=test_gyro, dt=dt)
    
    print(f"State after predict: {ekf.state}")
    print(f"State changed: {not np.allclose(initial_state, ekf.state)}")
    
    # Periksa apakah ada NaN atau Inf
    if np.any(np.isnan(ekf.state)) or np.any(np.isinf(ekf.state)):
        print("WARNING: NaN or Inf detected in state!")
    
    print("=== End EKF Debug ===\n")
    
    return ekf
    
def main():
    # Inisialisasi parameter
    dt = 0.1  # 10 Hz update rate

    # Inisialisasi handlers
    gps_handler = GPSHandler()
    imu_handler = IMUHandler()
    
    # Inisialisasi EKF
    ekf = EKFSensorFusion(dt=dt)
    
    # Debug EKF untuk memastikan berfungsi
    print("Debugging EKF initialization...")
    debug_ekf_initialization()
    
    # Setup logging
    log_queue = Queue()
    log_filename = f"sensor_fusion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_thread = Thread(target=logging_thread_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    # Variabel untuk tracking
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    
    # Inisialisasi EKF dengan nilai default (akan dikoreksi saat GPS tersedia)
    ekf.state[0] = 0      # x position (unknown)
    ekf.state[1] = 0      # y position (unknown)
    ekf.state[2] = 0      # heading (unknown initially)
    ekf.state[3] = 0      # angular velocity (akan diupdate dari IMU)
    ekf.state[4] = 0      # velocity (unknown initially)
    ekf.state[5] = 0      # acceleration (akan diupdate dari IMU)
    
    print("Starting sensor fusion...")
    print("EKF initialized - running in dead reckoning mode until GPS available")
    
    try:
        print("ekf loop running...")
        iteration_count = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            # Pastikan dt_actual masuk akal
            if dt_actual <= 0 or dt_actual > 1.0:
                dt_actual = dt  # fallback ke default
                
            # Baca data IMU (selalu tersedia)
            accel, gyro = imu_handler.get_data()
            
            # Debug print untuk beberapa iterasi pertama
            if iteration_count < 5:
                print(f"Iteration {iteration_count}: accel={accel:.3f}, gyro={gyro:.3f}, dt={dt_actual:.3f}")
            
            # PREDICT STEP - SELALU BERJALAN (dead reckoning dengan IMU)
            try:
                ekf.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                # Debug: print state setelah predict untuk beberapa iterasi pertama
                if iteration_count < 5:
                    print(f"  State after predict: x={ekf.state[0]:.3f}, y={ekf.state[1]:.3f}, "
                          f"heading={math.degrees(ekf.state[2]):.1f}°, vel={ekf.state[4]:.3f}")
                          
            except Exception as e:
                print(f"Error in EKF predict: {e}")
                continue
            
            # Baca data GPS dan validasi
            gps_coords = gps_handler.get_coords()
            gps_valid = is_gps_valid(gps_coords)

            # Setup referensi GPS jika belum ada DAN GPS valid
            if gps_valid and not gps_initialized:
                lat_ref = gps_coords['latitude']
                lon_ref = gps_coords['longitude']
                alt_ref = gps_coords.get('altitude', 0)
                
                # Reset posisi EKF ke GPS pertama (koreksi dari dead reckoning)
                x_gps, y_gps = latlon_to_xy(lat_ref, lon_ref, alt_ref, 
                                           lat_ref, lon_ref, alt_ref)
                ekf.state[0] = x_gps  # x position
                ekf.state[1] = y_gps  # y position
                # heading, velocity, acceleration tetap dari dead reckoning
                
                gps_initialized = True
                print(f"GPS reference set at lat: {lat_ref:.6f}, lon: {lon_ref:.6f}")
                print("Switching to GPS-aided navigation mode")
            
            # UPDATE STEP - hanya jika GPS valid dan sudah terinisialisasi
            if gps_valid and gps_initialized:
                # Konversi GPS ke koordinat lokal
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                
                # Update EKF dengan measurement GPS
                gps_measurement = np.array([gps_x, gps_y])
                ekf.update(gps_measurement)
                gps_update_counter += 1
            
            # Extract state untuk logging
            est_x, est_y, est_heading, est_omega, est_velocity, est_accel = ekf.state
            est_heading_deg = math.degrees(est_heading)
            
            # Logging data
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                log_data = [
                    current_time,
                    gps_x, gps_y,
                    gps_coords['latitude'], gps_coords['longitude'],
                    accel, gyro,
                    est_x, est_y, est_heading_deg, est_velocity, est_accel
                ]
            else:
                log_data = [
                    current_time,
                    None, None, None, None,
                    accel, gyro,
                    est_x, est_y, est_heading_deg, est_velocity, est_accel
                ]
            
            log_queue.put(log_data)
            
            # Print status lebih sering untuk debugging
            if iteration_count % 10 == 0:  # Print setiap 10 iterasi (1 detik pada 10Hz)
                if gps_initialized:
                    mode = "GPS-aided" if gps_valid else "GPS-aided (GPS signal lost)"
                else:
                    mode = "Dead reckoning (Waiting for GPS)"
                    
                print(f"Time: {current_time:.1f}s | Mode: {mode} | "
                      f"Pos: ({est_x:.3f}, {est_y:.3f}) | "
                      f"Heading: {est_heading_deg:.1f}° | "
                      f"Vel: {est_velocity:.3f} m/s | "
                      f"Accel: {est_accel:.3f} m/s² | "
                      f"GPS updates: {gps_update_counter}")
            
            iteration_count += 1
            
            last_time = current_time
            
            # Sleep untuk maintain update rate
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Cleanup
        gps_handler.stop()
        imu_handler.stop()
        log_queue.put("STOP")
        log_thread.join()
        print(f"Data logged to: {log_filename}")

if __name__ == "__main__":
    main()
