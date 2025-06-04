import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock
from queue import Queue
from gpsread import GPSReader
from dfrobotimu import IMU_WT61PCTTL
import pymap3d as pm
from ekfnparam2 import EKFSensorFusion
from mpu9250read import mpu9250
from mpu6050read import mpu6050

class GPSHandler:
    def __init__(self, port="/dev/ttyUSB0"):
        self.reader = GPSReader(port)
        self.latest_coords = None
        self.prev_coords = None
        self.running = True
        self.lock = Lock()  # Tambahkan lock untuk thread safety
        self.read_count = 0
        self.error_count = 0
        self.last_read_time = 0
        
        # Buffer untuk deteksi anomali
        self.coord_history = []
        self.max_history = 5
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        consecutive_errors = 0
        while self.running:
            try:
                # Tambahkan delay yang lebih konsisten
                time.sleep(0.1)  # 10Hz GPS reading
                
                coords = self.reader.read()
                current_time = time.time()
                
                if coords:
                    lat, lon = coords
                    new_coords = {
                        'latitude': lat,
                        'longitude': lon,
                        'altitude': 0,
                        'timestamp': current_time
                    }
                    
                    # Validasi koordinat
                    if self.validate_coordinates(new_coords):
                        with self.lock:
                            self.latest_coords = new_coords
                            self.prev_coords = self.latest_coords
                            self.read_count += 1
                            self.last_read_time = current_time
                        consecutive_errors = 0
                        
                        # Debug print
                        if self.read_count % 10 == 0:
                            print(f"GPS: Read #{self.read_count}, Lat: {lat:.6f}, Lon: {lon:.6f}, Errors: {self.error_count}")
                    else:
                        consecutive_errors += 1
                        self.error_count += 1
                        print(f"GPS: Invalid coordinates - Lat: {lat}, Lon: {lon} (Error #{self.error_count})")
                        
                        if consecutive_errors > 10:
                            print("GPS: Too many consecutive errors, attempting to reconnect...")
                            self.reconnect_gps()
                            consecutive_errors = 0
                else:
                    consecutive_errors += 1
                    if consecutive_errors % 20 == 0:
                        print(f"GPS: No data received for {consecutive_errors} cycles")
                        
            except Exception as e:
                self.error_count += 1
                print(f"GPS: Exception in update_loop: {e}")
                time.sleep(0.5)  # Wait longer on exception
    
    def validate_coordinates(self, coords):
        """Validasi koordinat GPS untuk mendeteksi pembacaan yang error"""
        lat = coords['latitude']
        lon = coords['longitude']
        
        # Basic range check
        if abs(lat) > 90 or abs(lon) > 180:
            return False
        if lat == 0.0 and lon == 0.0:
            return False
            
        # Check for sudden jumps
        if self.prev_coords:
            distance = self.calculate_distance(self.prev_coords, coords)
            # Jika bergerak lebih dari 100m dalam 0.1 detik (1000 m/s = 3600 km/h), reject
            time_diff = coords['timestamp'] - self.prev_coords.get('timestamp', 0)
            if time_diff > 0 and distance / time_diff > 50:  # Max 50 m/s = 180 km/h
                print(f"GPS: Rejecting jump - Distance: {distance:.2f}m in {time_diff:.2f}s = {distance/time_diff:.2f}m/s")
                return False
        
        return True
    
    def calculate_distance(self, coord1, coord2):
        """Hitung jarak antara dua koordinat GPS"""
        R = 6371000
        lat1, lon1 = math.radians(coord1['latitude']), math.radians(coord1['longitude'])
        lat2, lon2 = math.radians(coord2['latitude']), math.radians(coord2['longitude'])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def reconnect_gps(self):
        """Coba reconnect GPS"""
        try:
            self.reader.close()
            time.sleep(1)
            self.reader = GPSReader(self.reader.port)
        except Exception as e:
            print(f"GPS: Reconnection failed: {e}")
    
    def get_coords(self):
        with self.lock:
            return self.latest_coords
    
    def get_stats(self):
        return {
            'read_count': self.read_count,
            'error_count': self.error_count,
            'last_read_time': self.last_read_time
        }
    
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
        self.lock = Lock()  # Thread safety
        self.read_count = 0
        self.error_count = 0
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            try:
                # Beri delay yang berbeda dari GPS untuk menghindari collision
                time.sleep(0.02)  # 50Hz IMU reading
                
                data_accel = self.imu.get_accel_data()
                data_gyro = self.imu.get_gyro_data()
                
                with self.lock:
                    self.accel = data_accel['x']
                    self.gyro = math.radians(data_gyro['z'])
                    self.read_count += 1
                
                # Debug print
                if self.read_count % 100 == 0:
                    print(f"IMU: Read #{self.read_count}, Accel: {self.accel:.3f}, Gyro: {math.degrees(self.gyro):.3f}°/s")
                    
            except Exception as e:
                self.error_count += 1
                print(f"IMU: Exception in update_loop: {e}")
                time.sleep(0.1)
    
    def get_data(self):
        with self.lock:
            return self.accel, self.gyro
    
    def get_stats(self):
        return {
            'read_count': self.read_count,
            'error_count': self.error_count
        }
    
    def stop(self):
        self.running = False
        self.thread.join()

# Test function untuk isolasi masalah
def test_gps_only():
    """Test GPS saja tanpa IMU"""
    print("=== Testing GPS Only ===")
    gps_handler = GPSHandler()
    
    try:
        for i in range(100):
            coords = gps_handler.get_coords()
            if coords:
                print(f"GPS #{i}: Lat: {coords['latitude']:.6f}, Lon: {coords['longitude']:.6f}")
            else:
                print(f"GPS #{i}: No data")
            time.sleep(0.5)
    finally:
        gps_handler.stop()
        stats = gps_handler.get_stats()
        print(f"GPS Stats: Reads: {stats['read_count']}, Errors: {stats['error_count']}")

def test_imu_only():
    """Test IMU saja tanpa GPS"""
    print("=== Testing IMU Only ===")
    imu_handler = IMUHandler()
    
    try:
        for i in range(100):
            accel, gyro = imu_handler.get_data()
            print(f"IMU #{i}: Accel: {accel:.3f}, Gyro: {math.degrees(gyro):.3f}°/s")
            time.sleep(0.1)
    finally:
        imu_handler.stop()
        stats = imu_handler.get_stats()
        print(f"IMU Stats: Reads: {stats['read_count']}, Errors: {stats['error_count']}")

def test_both_sensors():
    """Test kedua sensor bersamaan dengan monitoring"""
    print("=== Testing Both Sensors ===")
    gps_handler = GPSHandler()
    imu_handler = IMUHandler()
    
    try:
        start_time = time.time()
        for i in range(200):
            current_time = time.time()
            
            coords = gps_handler.get_coords()
            accel, gyro = imu_handler.get_data()
            
            if i % 20 == 0:  # Print every 2 seconds
                if coords:
                    print(f"Time: {current_time-start_time:.1f}s | "
                          f"GPS: Lat: {coords['latitude']:.6f}, Lon: {coords['longitude']:.6f} | "
                          f"IMU: Accel: {accel:.3f}, Gyro: {math.degrees(gyro):.1f}°/s")
                else:
                    print(f"Time: {current_time-start_time:.1f}s | GPS: No data | "
                          f"IMU: Accel: {accel:.3f}, Gyro: {math.degrees(gyro):.1f}°/s")
            
            time.sleep(0.1)
    finally:
        gps_stats = gps_handler.get_stats()
        imu_stats = imu_handler.get_stats()
        
        print(f"\nFinal Stats:")
        print(f"GPS - Reads: {gps_stats['read_count']}, Errors: {gps_stats['error_count']}")
        print(f"IMU - Reads: {imu_stats['read_count']}, Errors: {imu_stats['error_count']}")
        
        gps_handler.stop()
        imu_handler.stop()

def main():
    print("GPS-IMU Interference Debug Tool")
    print("1. Test GPS only")
    print("2. Test IMU only") 
    print("3. Test both sensors")
    print("4. Run main sensor fusion")
    
    choice = input("Enter choice (1-4): ").strip()
    
    if choice == '1':
        test_gps_only()
    elif choice == '2':
        test_imu_only()
    elif choice == '3':
        test_both_sensors()
    elif choice == '4':
        # Original main function with improvements
        main_sensor_fusion()
    else:
        print("Invalid choice")

def main_sensor_fusion():
    """Main sensor fusion dengan perbaikan"""
    dt = 0.1
    
    # Stagger initialization untuk menghindari collision
    print("Initializing GPS...")
    gps_handler = GPSHandler()
    time.sleep(2)  # Beri waktu GPS untuk stabilize
    
    print("Initializing IMU...")
    imu_handler = IMUHandler()
    time.sleep(1)  # Beri waktu IMU untuk stabilize
    
    print("Initializing EKF...")
    ekf = EKFSensorFusion(dt=dt)
    
    # Setup logging
    log_queue = Queue()
    log_filename = f"sensor_fusion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_thread = Thread(target=logging_thread_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    # Variables
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    
    print("Starting sensor fusion with improved GPS handling...")
    
    try:
        iteration_count = 0
        target_dt = 0.02
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 1.0:
                dt_actual = dt
                
            # Read IMU data
            accel, gyro = imu_handler.get_data()
            
            # EKF Predict
            try:
                ekf.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
            except Exception as e:
                print(f"Error in EKF predict: {e}")
                continue
            
            # Read GPS data
            gps_coords = gps_handler.get_coords()
            gps_valid = is_gps_valid(gps_coords)
            
            # GPS initialization
            if gps_valid and not gps_initialized:
                lat_ref = gps_coords['latitude']
                lon_ref = gps_coords['longitude'] 
                alt_ref = gps_coords.get('altitude', 0)
                
                x_gps, y_gps = latlon_to_xy(lat_ref, lon_ref, alt_ref, 
                                           lat_ref, lon_ref, alt_ref)
                ekf.state[0] = x_gps
                ekf.state[1] = y_gps
                
                gps_initialized = True
                print(f"GPS reference set at lat: {lat_ref:.6f}, lon: {lon_ref:.6f}")
            
            # GPS update
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                
                gps_measurement = np.array([gps_x, gps_y])
                ekf.update(gps_measurement)
                gps_update_counter += 1
            
            # Extract state
            est_x, est_y, est_heading, est_omega, est_velocity, est_accel = ekf.state
            est_heading_deg = math.degrees(est_heading)
            
            # Logging
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                log_data = [
                    current_time, gps_x, gps_y,
                    gps_coords['latitude'], gps_coords['longitude'],
                    accel, gyro,
                    est_x, est_y, est_heading_deg, est_velocity, est_accel
                ]
            else:
                log_data = [
                    current_time, None, None, None, None,
                    accel, gyro,
                    est_x, est_y, est_heading_deg, est_velocity, est_accel
                ]
            
            log_queue.put(log_data)
            
            # Status print dengan sensor stats
            if iteration_count % 50 == 0:
                gps_stats = gps_handler.get_stats()
                imu_stats = imu_handler.get_stats()
                
                print(f"Time: {current_time:.1f}s | "
                      f"GPS: reads={gps_stats['read_count']}, errors={gps_stats['error_count']} | "
                      f"IMU: reads={imu_stats['read_count']}, errors={imu_stats['error_count']} | "
                      f"Pos: ({est_x:.3f}, {est_y:.3f}) | "
                      f"GPS updates: {gps_update_counter}")
            
            iteration_count += 1
            last_time = current_time
            
            # Sleep
            sleep_time = target_dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        gps_handler.stop()
        imu_handler.stop()
        log_queue.put("STOP")
        log_thread.join()
        print(f"Data logged to: {log_filename}")

# Supporting functions (unchanged)
def is_gps_valid(gps_coords):
    if not gps_coords:
        return False
    
    lat = gps_coords.get('latitude', 0)
    lon = gps_coords.get('longitude', 0)
    
    if (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
        return False
    
    return True

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
    return e, n

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

if __name__ == "__main__":
    main()
