import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock
from queue import Queue
import collections
from scipy import signal  # TAMBAHAN YANG HILANG!
from gpsread import GPSReader
from dfrobotimu import IMU_WT61PCTTL
import pymap3d as pm
from ekfnparam3 import EKFSensorFusion  # Import EKF Anda
from mpu9250read import mpu9250
from mpu6050read import mpu6050

# GPSHandler - DIPERBAIKI: Better error handling dan validation
class GPSHandler:
    def __init__(self, port="/dev/ttyUSB0"):
        self.reader = GPSReader(port)
        self.latest_coords = None
        self.prev_coords = None
        self.running = True
        self.lock = Lock()
        self.read_count = 0
        self.error_count = 0
        self.last_read_time = 0
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        consecutive_errors = 0
        max_consecutive_errors = 10  # TAMBAHAN: Limit consecutive errors
        
        while self.running:
            try:
                time.sleep(0.1)  # 10Hz GPS
                
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
                    
                    if self.validate_coordinates(new_coords):
                        with self.lock:
                            self.prev_coords = self.latest_coords  # PERBAIKAN: Update prev sebelum latest
                            self.latest_coords = new_coords
                            self.read_count += 1
                            self.last_read_time = current_time
                        consecutive_errors = 0
                    else:
                        consecutive_errors += 1
                        self.error_count += 1
                        
                        # TAMBAHAN: Reset jika terlalu banyak error berturut-turut
                        if consecutive_errors >= max_consecutive_errors:
                            print(f"GPS: Too many consecutive errors ({consecutive_errors}), resetting...")
                            time.sleep(1)
                            consecutive_errors = 0
                        
            except Exception as e:
                self.error_count += 1
                consecutive_errors += 1
                print(f"GPS Exception: {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    print("GPS: Critical error, sleeping longer...")
                    time.sleep(2)
                    consecutive_errors = 0
                else:
                    time.sleep(0.5)
    
    def validate_coordinates(self, coords):
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
            time_diff = coords['timestamp'] - self.prev_coords.get('timestamp', 0)
            if time_diff > 0 and distance / time_diff > 50:  # Max 50 m/s
                print(f"GPS: Rejecting jump: {distance:.1f}m in {time_diff:.1f}s = {distance/time_diff:.1f}m/s")
                return False
        
        return True
    
    def calculate_distance(self, coord1, coord2):
        R = 6371000
        lat1, lon1 = math.radians(coord1['latitude']), math.radians(coord1['longitude'])
        lat2, lon2 = math.radians(coord2['latitude']), math.radians(coord2['longitude'])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
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
        if self.thread.is_alive():
            self.thread.join()
        try:
            self.reader.close()
        except:
            pass

# PERBAIKAN: IMULowPassFilter class dengan error handling yang lebih baik
class IMULowPassFilter:
    """
    Real-time Butterworth low pass filter untuk IMU data
    """
    def __init__(self, accel_cutoff=15, gyro_cutoff=10, fs=50, order=2):
        """
        Args:
            accel_cutoff: Cutoff frequency untuk accelerometer (Hz)
            gyro_cutoff: Cutoff frequency untuk gyroscope (Hz) 
            fs: Sampling frequency (Hz)
            order: Filter order (2 atau 4 recommended)
        """
        self.fs = fs
        nyquist = 0.5 * fs
        
        # PERBAIKAN: Validasi parameter
        if accel_cutoff >= nyquist or gyro_cutoff >= nyquist:
            raise ValueError(f"Cutoff frequencies must be less than Nyquist frequency ({nyquist}Hz)")
        
        # Design Butterworth filters
        try:
            self.accel_b, self.accel_a = signal.butter(
                order, accel_cutoff/nyquist, btype='low', analog=False)
            self.gyro_b, self.gyro_a = signal.butter(
                order, gyro_cutoff/nyquist, btype='low', analog=False)
        except Exception as e:
            print(f"Filter design error: {e}")
            raise
        
        # Initialize filter states (untuk real-time filtering)
        self.accel_zi = signal.lfilter_zi(self.accel_b, self.accel_a)
        self.gyro_zi = signal.lfilter_zi(self.gyro_b, self.gyro_a)
        
        # Buffer untuk startup (opsional, untuk stabilitas awal)
        self.startup_buffer_size = 10
        self.accel_buffer = collections.deque(maxlen=self.startup_buffer_size)
        self.gyro_buffer = collections.deque(maxlen=self.startup_buffer_size)
        self.is_initialized = False
        
        print(f"Filter initialized: Accel={accel_cutoff}Hz, Gyro={gyro_cutoff}Hz, FS={fs}Hz, Order={order}")
        
    def filter_imu_data(self, accel_raw, gyro_raw):
        """
        Apply low pass filter to IMU data
        
        Args:
            accel_raw: Raw accelerometer reading
            gyro_raw: Raw gyroscope reading
            
        Returns:
            tuple: (filtered_accel, filtered_gyro)
        """
        
        # PERBAIKAN: Input validation
        if not isinstance(accel_raw, (int, float)) or not isinstance(gyro_raw, (int, float)):
            print(f"Invalid IMU data type: accel={type(accel_raw)}, gyro={type(gyro_raw)}")
            return 0.0, 0.0
        
        if math.isnan(accel_raw) or math.isnan(gyro_raw) or math.isinf(accel_raw) or math.isinf(gyro_raw):
            print(f"Invalid IMU data values: accel={accel_raw}, gyro={gyro_raw}")
            return 0.0, 0.0
        
        # Startup phase - collect data untuk stabilitas
        if not self.is_initialized:
            self.accel_buffer.append(accel_raw)
            self.gyro_buffer.append(gyro_raw)
            
            if len(self.accel_buffer) >= self.startup_buffer_size:
                # Initialize dengan rata-rata buffer
                avg_accel = sum(self.accel_buffer) / len(self.accel_buffer)
                avg_gyro = sum(self.gyro_buffer) / len(self.gyro_buffer)
                
                # Set initial conditions
                self.accel_zi = self.accel_zi * avg_accel
                self.gyro_zi = self.gyro_zi * avg_gyro
                self.is_initialized = True
                
                print(f"Filter initialized with startup averages: A={avg_accel:.3f}, G={avg_gyro:.3f}")
                return avg_accel, avg_gyro
            else:
                # Return moving average selama startup
                return (sum(self.accel_buffer) / len(self.accel_buffer),
                       sum(self.gyro_buffer) / len(self.gyro_buffer))
        
        # Real-time filtering setelah initialized
        try:
            accel_filtered, self.accel_zi = signal.lfilter(
                self.accel_b, self.accel_a, [accel_raw], zi=self.accel_zi)
            
            gyro_filtered, self.gyro_zi = signal.lfilter(
                self.gyro_b, self.gyro_a, [gyro_raw], zi=self.gyro_zi)
            
            return float(accel_filtered[0]), float(gyro_filtered[0])
            
        except Exception as e:
            print(f"Filter processing error: {e}")
            return accel_raw, gyro_raw  # Fallback ke raw data
    
    def reset(self):
        """Reset filter states"""
        self.accel_zi = signal.lfilter_zi(self.accel_b, self.accel_a)
        self.gyro_zi = signal.lfilter_zi(self.gyro_b, self.gyro_a)
        self.accel_buffer.clear()
        self.gyro_buffer.clear()
        self.is_initialized = False
        print("Filter reset")

# PERBAIKAN: IMUHandler class dengan better error handling
class IMUHandler:
    def __init__(self, i2c=0x68, enable_filter=True, accel_cutoff=15, gyro_cutoff=10):
        try:
            self.imu = mpu9250(i2c)
            self.imu.calibrate()
            print(f"IMU initialized successfully at address 0x{i2c:02x}")
        except Exception as e:
            print(f"IMU initialization failed: {e}")
            raise
            
        self.accel = 0
        self.gyro = 0
        self.running = True
        self.lock = Lock()
        self.read_count = 0
        self.error_count = 0
        
        # Low pass filter setup
        self.enable_filter = enable_filter
        if self.enable_filter:
            try:
                self.filter = IMULowPassFilter(
                    accel_cutoff=accel_cutoff, 
                    gyro_cutoff=gyro_cutoff,
                    fs=50,  # IMU sampling rate
                    order=2
                )
                print(f"IMU Filter enabled: Accel cutoff={accel_cutoff}Hz, Gyro cutoff={gyro_cutoff}Hz")
            except Exception as e:
                print(f"Filter initialization failed: {e}")
                self.enable_filter = False
                print("Falling back to raw IMU data")
        else:
            print("IMU Filter disabled - using raw data")
        
        # Raw data storage (untuk debugging)
        self.accel_raw = 0
        self.gyro_raw = 0
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        consecutive_errors = 0
        max_consecutive_errors = 20
        
        while self.running:
            try:
                time.sleep(0.02)  # 50Hz IMU
                
                data_accel = self.imu.get_accel_data()
                data_gyro = self.imu.get_gyro_data()
                
                # PERBAIKAN: Validasi data dari IMU
                if not data_accel or not data_gyro:
                    raise ValueError("IMU returned empty data")
                
                accel_raw = data_accel.get('x', 0)
                gyro_raw = math.radians(data_gyro.get('z', 0))
                
                # PERBAIKAN: Validasi nilai
                if math.isnan(accel_raw) or math.isnan(gyro_raw):
                    raise ValueError(f"IMU returned NaN values: accel={accel_raw}, gyro={gyro_raw}")
                
                # Apply filter jika enabled
                if self.enable_filter:
                    try:
                        accel_filtered, gyro_filtered = self.filter.filter_imu_data(accel_raw, gyro_raw)
                    except Exception as e:
                        print(f"Filter error: {e}")
                        accel_filtered, gyro_filtered = accel_raw, gyro_raw
                else:
                    accel_filtered, gyro_filtered = accel_raw, gyro_raw
                
                with self.lock:
                    self.accel_raw = accel_raw
                    self.gyro_raw = gyro_raw
                    self.accel = accel_filtered
                    self.gyro = gyro_filtered
                    self.read_count += 1
                
                consecutive_errors = 0  # Reset pada successful read
                    
            except Exception as e:
                self.error_count += 1
                consecutive_errors += 1
                
                if consecutive_errors <= 3:  # Print hanya beberapa error pertama
                    print(f"IMU Exception: {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    print(f"IMU: Too many consecutive errors ({consecutive_errors}), attempting recovery...")
                    try:
                        # Attempt IMU recovery
                        self.imu.calibrate()
                        consecutive_errors = 0
                        time.sleep(1)
                    except:
                        print("IMU recovery failed")
                        time.sleep(2)
                else:
                    time.sleep(0.1)
    
    def get_data(self):
        """Get filtered IMU data"""
        with self.lock:
            return self.accel, self.gyro
    
    def get_raw_data(self):
        """Get raw IMU data (untuk comparison)"""
        with self.lock:
            return self.accel_raw, self.gyro_raw
    
    def get_stats(self):
        return {
            'read_count': self.read_count,
            'error_count': self.error_count
        }
    
    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=3)  # PERBAIKAN: Timeout untuk join

# PERBAIKAN: Main sensor fusion function dengan better error handling
def main_sensor_fusion(enable_filter=True, accel_cutoff=15, gyro_cutoff=10):
    """Main sensor fusion dengan optional low pass filter"""
    
    if enable_filter:
        print(f"=== SENSOR FUSION WITH LOW PASS FILTER ===")
        print(f"Accel cutoff: {accel_cutoff}Hz, Gyro cutoff: {gyro_cutoff}Hz")
    else:
        print("=== SENSOR FUSION WITHOUT FILTER (RAW IMU) ===")
    
    # Initialize dengan delay untuk stabilitas
    print("Initializing GPS...")
    try:
        gps_handler = GPSHandler()
        time.sleep(2)
        print("GPS initialized successfully")
    except Exception as e:
        print(f"GPS initialization failed: {e}")
        return
    
    print("Initializing IMU...")
    try:
        imu_handler = IMUHandler(
            enable_filter=enable_filter,
            accel_cutoff=accel_cutoff,
            gyro_cutoff=gyro_cutoff
        )
        time.sleep(2)  # Extra delay untuk filter startup
        print("IMU initialized successfully")
    except Exception as e:
        print(f"IMU initialization failed: {e}")
        gps_handler.stop()
        return
    
    print("Initializing EKF...")
    try:
        dt = 0.05  # 20Hz main loop
        ekf = EKFSensorFusion(dt=dt)
        print("EKF initialized successfully")
    except Exception as e:
        print(f"EKF initialization failed: {e}")
        gps_handler.stop()
        imu_handler.stop()
        return
    
    # Setup enhanced logging dengan filter data
    log_queue = Queue()
    imu_predict_queue = Queue()
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filter_suffix = f"_filtered_{accel_cutoff}_{gyro_cutoff}" if enable_filter else "_raw"
    log_filename = f"sensor_fusion{filter_suffix}_{timestamp}.csv"
    imu_predict_filename = f"imu_predict{filter_suffix}_{timestamp}.csv"
    
    # Logging threads
    log_thread = Thread(target=enhanced_logging_thread_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    imu_predict_thread = Thread(target=enhanced_imu_predict_logging_func, args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    # Variables
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    predict_counter = 0
    
    print("Starting sensor fusion...")
    print(f"Combined log: {log_filename}")
    print(f"IMU predict log: {imu_predict_filename}")
    
    try:
        iteration_count = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            # Clamp dt untuk stabilitas
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # Read IMU data (filtered dan raw)
            try:
                accel, gyro = imu_handler.get_data()  # Filtered
                accel_raw, gyro_raw = imu_handler.get_raw_data()  # Raw
                
                # PERBAIKAN: Validasi IMU data
                if math.isnan(accel) or math.isnan(gyro) or math.isinf(accel) or math.isinf(gyro):
                    print("Invalid IMU data, using zeros")
                    accel, gyro = 0, 0
                if math.isnan(accel_raw) or math.isnan(gyro_raw) or math.isinf(accel_raw) or math.isinf(gyro_raw):
                    accel_raw, gyro_raw = 0, 0
                    
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
                accel_raw, gyro_raw = 0, 0
            
            # EKF Predict dengan filtered IMU data
            try:
                ekf.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                predict_counter += 1
                
                # PERBAIKAN: Better state extraction dengan error handling
                try:
                    if len(ekf.state) == 6:
                        pred_x, pred_y, pred_heading, pred_omega, pred_velocity, pred_accel = ekf.state
                    elif len(ekf.state) == 4:
                        pred_x, pred_y, pred_heading, pred_velocity = ekf.state
                        pred_omega = gyro
                        pred_accel = accel
                    else:
                        print(f"⚠️ Unexpected EKF state dimension: {len(ekf.state)}")
                        break
                    
                    # PERBAIKAN: Validasi state values
                    if any(math.isnan(val) or math.isinf(val) for val in [pred_x, pred_y, pred_heading, pred_velocity]):
                        print("⚠️ EKF state contains invalid values, resetting...")
                        ekf = EKFSensorFusion(dt=dt)  # Reset EKF
                        continue
                        
                    pred_heading_deg = math.degrees(pred_heading)
                    
                    # Enhanced logging dengan raw dan filtered data
                    imu_predict_data = [
                        current_time, dt_actual,
                        accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw data
                        accel, gyro, math.degrees(gyro),              # Filtered data
                        pred_x, pred_y, pred_heading_deg, pred_velocity, pred_accel,
                        predict_counter
                    ]
                    imu_predict_queue.put(imu_predict_data)
                    
                except Exception as e:
                    print(f"State extraction error: {e}")
                    continue
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                last_time = current_time
                continue
            
            # GPS handling
            try:
                gps_coords = gps_handler.get_coords()
                gps_valid = is_gps_valid(gps_coords)
                gps_updated = False
                
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
                    print(f"GPS reference set: {lat_ref:.6f}, {lon_ref:.6f}")
                
                # GPS update
                if gps_valid and gps_initialized:
                    try:
                        gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                                   gps_coords['longitude'],
                                                   gps_coords.get('altitude', alt_ref),
                                                   lat_ref, lon_ref, alt_ref)
                        
                        gps_measurement = np.array([gps_x, gps_y])
                        ekf.update(gps_measurement)
                        gps_update_counter += 1
                        gps_updated = True
                    except Exception as e:
                        print(f"GPS update error: {e}")
                        
            except Exception as e:
                print(f"GPS handling error: {e}")
            
            # Extract final state
            try:
                if len(ekf.state) == 6:
                    est_x, est_y, est_heading, est_omega, est_velocity, est_accel = ekf.state
                elif len(ekf.state) == 4:
                    est_x, est_y, est_heading, est_velocity = ekf.state
                    est_omega = gyro
                    est_accel = accel
                    
                est_heading_deg = math.degrees(est_heading)
                
            except Exception as e:
                print(f"Final state extraction error: {e}")
                continue
            
            # Enhanced combined logging
            try:
                if gps_valid and gps_initialized:
                    gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                               gps_coords['longitude'],
                                               gps_coords.get('altitude', alt_ref),
                                               lat_ref, lon_ref, alt_ref)
                    log_data = [
                        current_time, dt_actual,
                        gps_x, gps_y, gps_coords['latitude'], gps_coords['longitude'],
                        accel_raw, gyro_raw, math.degrees(gyro_raw),    # Raw IMU
                        accel, gyro, math.degrees(gyro),                # Filtered IMU
                        est_x, est_y, est_heading_deg, est_velocity, est_accel,
                        gps_updated, predict_counter, gps_update_counter
                    ]
                else:
                    log_data = [
                        current_time, dt_actual,
                        None, None, None, None,
                        accel_raw, gyro_raw, math.degrees(gyro_raw),
                        accel, gyro, math.degrees(gyro),
                        est_x, est_y, est_heading_deg, est_velocity, est_accel,
                        False, predict_counter, gps_update_counter
                    ]
                
                log_queue.put(log_data)
            except Exception as e:
                print(f"Logging error: {e}")
            
            # Enhanced status print dengan filter info
            if iteration_count % 40 == 0:
                try:
                    gps_stats = gps_handler.get_stats()
                    imu_stats = imu_handler.get_stats()
                    
                    filter_info = f" | Filter: {'ON' if enable_filter else 'OFF'}"
                    noise_info = ""
                    if enable_filter:
                        # Hitung simple noise metric
                        accel_diff = abs(accel - accel_raw)
                        gyro_diff = abs(gyro - gyro_raw)
                        noise_info = f" | Noise Δ: A={accel_diff:.3f} G={math.degrees(gyro_diff):.1f}°"
                    
                    print(f"t={iteration_count*dt:.1f}s | "
                          f"GPS: {gps_stats['read_count']}/{gps_stats['error_count']} | "
                          f"IMU: {imu_stats['read_count']}/{imu_stats['error_count']} | "
                          f"Predicts: {predict_counter} | GPS updates: {gps_update_counter} | "
                          f"Pos: ({est_x:.2f}, {est_y:.2f}) | "
                          f"V: {est_velocity:.2f} m/s | "
                          f"H: {est_heading_deg:.1f}°{filter_info}{noise_info}")
                    
                    # Warning checks
                    if abs(est_velocity) > 30:
                        print("⚠️ Unrealistic velocity!")
                    if hasattr(ekf, 'P') and np.trace(ekf.P) > 1000:
                        print("⚠️ EKF covariance exploding!")
                        
                except Exception as e:
                    print(f"Status print error: {e}")
            
            iteration_count += 1
            last_time = current_time
            
            # Consistent timing
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Main loop error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        gps_handler.stop()
        imu_handler.stop()
        
        # Stop logging threads
        log_queue.put("STOP")
        imu_predict_queue.put("STOP")
        
        if log_thread.is_alive():
            log_thread.join(timeout=2)
        if imu_predict_thread.is_alive():
            imu_predict_thread.join(timeout=2)
            
        print(f"Combined data logged to: {log_filename}")
        print(f"IMU predict data logged to: {imu_predict_filename}")

# FUNGSI HELPER - tidak berubah dari kode asli
def is_gps_valid(gps_coords):
    if not gps_coords:
        return False
    
    lat = gps_coords.get('latitude', 0)
    lon = gps_coords.get('longitude', 0)
    
    if (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
        return False
    
    return True

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Simple flat earth conversion"""
    R = 6371000  # Earth radius in meters
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat_ref_rad = math.radians(lat_ref)
    lon_ref_rad = math.radians(lon_ref)
    
    x = R * (lon_rad - lon_ref_rad) * math.cos(lat_ref_rad)
    y = R * (lat_rad - lat_ref_rad)
    
    return x, y

def logging_thread_func(q: Queue, filename: str):
    """Combined logging function"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel', 'gyro_rad', 'gyro_deg',
            'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
            'gps_updated', 'predict_count', 'gps_update_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def imu_predict_logging_func(q: Queue, filename: str):
    """IMU predict only logging function - INI YANG BARU!"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'accel', 'gyro_rad', 'gyro_deg',
            'predict_x', 'predict_y', 'predict_heading_deg', 
            'predict_velocity', 'predict_accel', 'predict_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def test_imu_predict_only():
    """Test khusus untuk melihat IMU predict tanpa GPS"""
    print("=== Testing IMU Predict Only (No GPS Updates) ===")
    
    print("Initializing IMU...")
    imu_handler = IMUHandler()
    time.sleep(1)
    
    print("Initializing EKF...")
    dt = 0.05
    ekf = EKFSensorFusion(dt=dt)
    
    # Set initial position manually
    ekf.state[0] = 0  # x
    ekf.state[1] = 0  # y
    
    # Setup logging
    imu_predict_queue = Queue()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    imu_predict_filename = f"imu_only_test_{timestamp}.csv"
    
    imu_predict_thread = Thread(target=imu_predict_logging_func, args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    print(f"IMU-only test logging to: {imu_predict_filename}")
    
    try:
        last_time = time.time()
        predict_counter = 0
        
        for i in range(1000):  # 50 detik test
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # Read IMU
            try:
                accel, gyro = imu_handler.get_data()
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
            
            # EKF Predict only
            try:
                ekf.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                predict_counter += 1
                
                # Extract state
                if len(ekf.state) == 6:
                    pred_x, pred_y, pred_heading, pred_omega, pred_velocity, pred_accel = ekf.state
                elif len(ekf.state) == 4:
                    pred_x, pred_y, pred_heading, pred_velocity = ekf.state
                    pred_omega = gyro
                    pred_accel = accel
                
                pred_heading_deg = math.degrees(pred_heading)
                
                # Log data
                imu_predict_data = [
                    current_time, dt_actual, accel, gyro, math.degrees(gyro),
                    pred_x, pred_y, pred_heading_deg, pred_velocity, pred_accel,
                    predict_counter
                ]
                imu_predict_queue.put(imu_predict_data)
                
                # Print status
                if i % 20 == 0:
                    print(f"IMU #{i}: Pos({pred_x:.2f}, {pred_y:.2f}) "
                          f"V:{pred_velocity:.2f} H:{pred_heading_deg:.1f}° "
                          f"A:{accel:.3f} Ω:{math.degrees(gyro):.1f}°/s")
                
            except Exception as e:
                print(f"EKF predict error: {e}")
            
            last_time = current_time
            
            # Timing
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        print("Cleaning up...")
        imu_handler.stop()
        imu_predict_queue.put("STOP")
        if imu_predict_thread.is_alive():
            imu_predict_thread.join(timeout=2)
        print(f"IMU-only test data logged to: {imu_predict_filename}")

# Test functions (unchanged from your code)
def test_gps_only():
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

def test_imu_only():
    print("=== Testing IMU Only ===")
    imu_handler = IMUHandler()
    
    try:
        for i in range(100):
            accel, gyro = imu_handler.get_data()
            print(f"IMU #{i}: Accel: {accel:.3f}, Gyro: {math.degrees(gyro):.3f}°/s")
            time.sleep(0.1)
    finally:
        imu_handler.stop()

def test_ekf_dimensions():
    """Test untuk cek dimensi EKF state vector"""
    print("=== Testing EKF Dimensions ===")
    ekf = EKFSensorFusion(dt=0.1)
    
    print(f"EKF state vector dimension: {len(ekf.state)}")
    print(f"EKF state vector: {ekf.state}")
    
    # Test predict
    ekf.predict(1.0, 0.1)
    print(f"After predict: {ekf.state}")
    
    if len(ekf.state) == 6:
        print("✓ Using 6D state vector [x, y, phi, omega, v, a]")
    elif len(ekf.state) == 4:
        print("✓ Using 4D state vector [x, y, phi, v]")
    else:
        print(f"⚠️ Unexpected dimension: {len(ekf.state)}")

# Enhanced logging functions
def enhanced_logging_thread_func(q: Queue, filename: str):
    """Enhanced combined logging dengan raw dan filtered IMU data"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
            'gps_updated', 'predict_count', 'gps_update_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def enhanced_imu_predict_logging_func(q: Queue, filename: str):
    """Enhanced IMU predict logging dengan raw dan filtered data"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'predict_x', 'predict_y', 'predict_heading_deg', 
            'predict_velocity', 'predict_accel', 'predict_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

# Test function untuk compare filter vs no filter
def test_filter_comparison():
    """Test untuk membandingkan dengan dan tanpa filter"""
    print("=== FILTER COMPARISON TEST ===")
    
    # Test 1: Tanpa filter
    print("\n1. Testing WITHOUT filter...")
    main_sensor_fusion(enable_filter=False)
    
    input("\nPress Enter to continue with filtered test...")
    
    # Test 2: Dengan filter
    print("\n2. Testing WITH filter...")
    main_sensor_fusion(enable_filter=True, accel_cutoff=15, gyro_cutoff=10)

# Modifikasi main function
def main():
    print("GPS-IMU Sensor Fusion with Low Pass Filter")
    print("1. Test GPS only")
    print("2. Test IMU only") 
    print("3. Test EKF dimensions")
    print("4. Run sensor fusion WITHOUT filter (raw IMU)")
    print("5. Run sensor fusion WITH filter (default: 15Hz/10Hz)")
    print("6. Run sensor fusion WITH custom filter")
    print("7. Test IMU predict only (no GPS)")
    print("8. Compare filter vs no filter")
    
    choice = input("Enter choice (1-8): ").strip()
    
    if choice == '1':
        test_gps_only()
    elif choice == '2':
        test_imu_only()
    elif choice == '3':
        test_ekf_dimensions()
    elif choice == '4':
        main_sensor_fusion(enable_filter=False)
    elif choice == '5':
        main_sensor_fusion(enable_filter=True)
    elif choice == '6':
        accel_cutoff = float(input("Accel cutoff frequency (Hz, default 15): ") or "15")
        gyro_cutoff = float(input("Gyro cutoff frequency (Hz, default 10): ") or "10")
        main_sensor_fusion(enable_filter=True, accel_cutoff=accel_cutoff, gyro_cutoff=gyro_cutoff)
    elif choice == '7':
        test_imu_predict_only()
    elif choice == '8':
        test_filter_comparison()
    else:
        print("Invalid choice")

if __name__ == "__main__":
    main()