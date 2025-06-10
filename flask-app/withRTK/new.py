import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock
from queue import Queue
import threading
from io import BytesIO

# Import custom modules
from gpsimuhandler import GPSHandler, FilteredIMUHandler, create_imu_handler
from ekfnparam3 import EKFSensorFusion
from helper import latlon_to_xy, is_gps_valid

# Import RTK class dari kode Anda
from rtkrcv import RTKRCVGPSCorrector  # Sesuaikan nama file
from ntripclient import NtripClient

class RTKGroundTruthLogger:
    """
    Class untuk mengelola RTK data sebagai ground truth dalam sistem EKF fusion
    """
    
    def __init__(self, rtk_corrector=None):
        self.rtk_corrector = rtk_corrector
        self.rtk_data = {}
        self.rtk_lock = threading.Lock()
        self.rtk_initialized = False
        self.rtk_ref_lat = None
        self.rtk_ref_lon = None
        self.rtk_ref_alt = None
        
    def get_rtk_position(self):
        """Mendapatkan posisi RTK terbaru dalam koordinat lokal"""
        if not self.rtk_corrector:
            return None
            
        with self.rtk_corrector.gps_lock:
            rtk_solution = self.rtk_corrector.rtk_solution.copy()
            
        if not rtk_solution:
            return None
            
        # Convert ke koordinat lokal jika sudah ada referensi
        if self.rtk_initialized and self.rtk_ref_lat is not None:
            try:
                rtk_x, rtk_y = latlon_to_xy(
                    rtk_solution['latitude'], 
                    rtk_solution['longitude'],
                    rtk_solution.get('altitude', self.rtk_ref_alt),
                    self.rtk_ref_lat, 
                    self.rtk_ref_lon, 
                    self.rtk_ref_alt
                )
                
                return {
                    'x': rtk_x,
                    'y': rtk_y,
                    'latitude': rtk_solution['latitude'],
                    'longitude': rtk_solution['longitude'],
                    'altitude': rtk_solution.get('altitude', 0),
                    'status': rtk_solution.get('status', 'Unknown'),
                    'quality': rtk_solution.get('quality', 0),
                    'satellites': rtk_solution.get('satellites', 0),
                    'std_north': rtk_solution.get('std_north', 0),
                    'std_east': rtk_solution.get('std_east', 0),
                    'std_up': rtk_solution.get('std_up', 0),
                    'timestamp': rtk_solution.get('timestamp', time.time()),
                    'rtcm_count': rtk_solution.get('rtcm_count', 0)
                }
            except Exception as e:
                print(f"RTK coordinate conversion error: {e}")
                return None
        else:
            # Return raw coordinates untuk initialization
            return {
                'latitude': rtk_solution['latitude'],
                'longitude': rtk_solution['longitude'],
                'altitude': rtk_solution.get('altitude', 0),
                'status': rtk_solution.get('status', 'Unknown'),
                'quality': rtk_solution.get('quality', 0),
                'raw_data': True
            }
    
    def set_reference(self, lat, lon, alt=0):
        """Set reference point untuk koordinat lokal RTK"""
        self.rtk_ref_lat = lat
        self.rtk_ref_lon = lon
        self.rtk_ref_alt = alt
        self.rtk_initialized = True
        print(f"RTK reference set: {lat:.6f}, {lon:.6f}, {alt:.1f}m")
    
    def get_rtk_status(self):
        """Mendapatkan status RTK secara umum"""
        if not self.rtk_corrector:
            return "RTK Not Available"
        
        with self.rtk_corrector.gps_lock:
            if self.rtk_corrector.rtk_solution:
                return self.rtk_corrector.rtk_solution.get('status', 'Unknown')
            else:
                return f"Processing (RTCM: {self.rtk_corrector.rtcm_count})"

def main_sensor_fusion_with_rtk_ground_truth():
    """
    Main sensor fusion dengan RTK sebagai ground truth logging
    RTK tidak digunakan dalam EKF, hanya sebagai referensi akurasi
    """
    
    print("=== SENSOR FUSION WITH RTK GROUND TRUTH ===")
    print("EKF menggunakan GPS biasa, RTK sebagai ground truth untuk evaluasi")
    
    # ===== RTK CONFIGURATION =====
    print("Initializing RTK system for ground truth...")
    
    # Initialize RTK corrector
    rtk_corrector = RTKLIBGPSCorrector(gps_port='/dev/ttyUSB0', gps_baudrate=115200)
    rtk_logger = RTKGroundTruthLogger(rtk_corrector)
    
    # NTRIP Configuration - sesuaikan dengan setup Anda
    ntrip_config = {
        'user': 'toor:toor123456',
        'caster': 'nrtk.big.go.id',
        'port': 2001,
        'mountpoint': '/vrs-rtcm3',
        'lat': -7.2839114,  # Koordinat perkiraan Anda
        'lon': 112.7961259,
        'height': 10.0,
        'verbose': False,
        'out': BytesIO()
    }
    
    # Modify NTRIP client untuk RTK corrector
    def modified_receive_data(client_self):
        lastGGATime = time.time()
        try:
            while True:
                current_time = time.time()
                
                # Update posisi dari GPS raw data
                if rtk_corrector.raw_gps_data:
                    with rtk_corrector.gps_lock:
                        client_self.setPosition(
                            rtk_corrector.raw_gps_data['latitude'],
                            rtk_corrector.raw_gps_data['longitude']
                        )
                        client_self.height = rtk_corrector.raw_gps_data['altitude']
                
                if current_time - lastGGATime > 10:
                    client_self.socket.send(client_self.getGGAString().encode())
                    lastGGATime = current_time
                
                client_self.socket.settimeout(1.0)
                try:
                    data = client_self.socket.recv(client_self.buffer)
                    if not data:
                        break
                    rtk_corrector.on_rtcm_data(data)
                except Exception:
                    continue
        except KeyboardInterrupt:
            print("NTRIP stopped by user")
        finally:
            if client_self.socket:
                client_self.socket.close()
    
    # Start RTK system
    print("Starting RTK GPS data collection...")
    rtk_gps_thread = threading.Thread(target=rtk_corrector.read_gps_data)
    rtk_gps_thread.daemon = True
    rtk_gps_thread.start()
    
    print("Starting NTRIP client...")
    NtripClient.receiveData = modified_receive_data
    ntrip_client = NtripClient(**ntrip_config)
    ntrip_thread = threading.Thread(target=ntrip_client.readLoop)
    ntrip_thread.daemon = True
    ntrip_thread.start()
    
    time.sleep(3)  # Beri waktu RTK untuk stabilize
    
    # ===== FILTER CONFIGURATION =====
    filter_config = {
        'application_type': 'vehicle',
        'accel_filter_config': {
            'cutoff': 3.0,
            'type': 'butterworth',
            'adaptive': False
        },
        'gyro_filter_config': {
            'cutoff': 5.0,
            'type': 'butterworth',
            'adaptive': False
        }
    }
    
    # Initialize standard GPS/IMU
    print("Initializing standard GPS...")
    gps_handler = GPSHandler()
    time.sleep(2)
    
    print("Initializing IMU with filtering...")
    imu_handler = FilteredIMUHandler(**filter_config)
    time.sleep(2)
    
    print("Initializing EKF...")
    dt = 0.05  # 20Hz
    ekf_combined = EKFSensorFusion(dt=dt)
    ekf_imu_only = EKFSensorFusion(dt=dt)
    
    # Setup enhanced logging dengan RTK ground truth
    log_queue = Queue()
    rtk_comparison_queue = Queue()
    imu_predict_queue = Queue()
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f"sensor_fusion_with_rtk_gt_{timestamp}.csv"
    rtk_comparison_filename = f"rtk_ground_truth_comparison_{timestamp}.csv"
    imu_predict_filename = f"imu_predict_with_rtk_{timestamp}.csv"
    
    # Enhanced logging threads
    log_thread = Thread(target=enhanced_logging_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    rtk_comparison_thread = Thread(target=rtk_comparison_logging_func, 
                                 args=(rtk_comparison_queue, rtk_comparison_filename))
    rtk_comparison_thread.daemon = True
    rtk_comparison_thread.start()
    
    imu_predict_thread = Thread(target=imu_predict_logging_func, 
                               args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    # Variables
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    predict_counter = 0
    rtk_fix_count = 0
    rtk_float_count = 0
    
    print(f"\nStarting sensor fusion with RTK ground truth...")
    print(f"Main log: {log_filename}")
    print(f"RTK comparison: {rtk_comparison_filename}")
    print(f"IMU predict: {imu_predict_filename}")
    print("RTK akan digunakan sebagai ground truth untuk evaluasi akurasi EKF")
    
    try:
        iteration_count = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # ===== READ FILTERED IMU DATA =====
            try:
                accel_filtered, gyro_filtered, accel_raw, gyro_raw, filter_stats = imu_handler.get_data(
                    return_raw=True, return_stats=True
                )
                accel = accel_filtered
                gyro = gyro_filtered
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
                accel_raw, gyro_raw = 0, 0
                filter_stats = {'accel_noise_reduction': 0, 'gyro_noise_reduction': 0}
            
            # ===== EKF PREDICT =====
            try:
                ekf_combined.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                ekf_imu_only.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                predict_counter += 1
                
                # Extract IMU-only prediction
                pure_predict_state = ekf_imu_only.state.copy()
                if len(pure_predict_state) >= 4:
                    pure_pred_x, pure_pred_y, pure_pred_heading, pure_pred_velocity = pure_predict_state[:4]
                    pure_pred_accel = accel
                else:
                    pure_pred_x, pure_pred_y = pure_predict_state[0], pure_predict_state[1]
                    pure_pred_heading = 0
                    pure_pred_velocity = 0
                    pure_pred_accel = accel
                
                pure_pred_heading_deg = math.degrees(pure_pred_heading)
                
                # Log IMU predict dengan RTK comparison jika tersedia
                rtk_position = rtk_logger.get_rtk_position()
                rtk_x, rtk_y = None, None
                rtk_status = "No RTK"
                
                if rtk_position and not rtk_position.get('raw_data', False):
                    rtk_x = rtk_position['x']
                    rtk_y = rtk_position['y']
                    rtk_status = rtk_position['status']
                
                imu_predict_data = [
                    current_time, dt_actual,
                    accel, gyro, math.degrees(gyro),
                    pure_pred_x, pure_pred_y, pure_pred_heading_deg, pure_pred_velocity, pure_pred_accel,
                    rtk_x, rtk_y, rtk_status,  # RTK ground truth
                    abs(pure_pred_x - rtk_x) if rtk_x is not None else None,  # Error dari RTK
                    abs(pure_pred_y - rtk_y) if rtk_y is not None else None,
                    predict_counter
                ]
                imu_predict_queue.put(imu_predict_data)
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                last_time = current_time
                continue
            
            # ===== READ STANDARD GPS =====
            gps_coords = gps_handler.get_coords()
            gps_valid = is_gps_valid(gps_coords)
            gps_updated = False
            
            # ===== READ RTK DATA =====
            rtk_position = rtk_logger.get_rtk_position()
            
            # GPS/RTK initialization dengan prioritas RTK jika tersedia
            if not gps_initialized:
                # Coba gunakan RTK untuk reference jika tersedia dan fixed
                if (rtk_position and rtk_position.get('raw_data') and 
                    rtk_position.get('quality', 0) in [1, 2]):  # RTK Fixed atau Float
                    
                    lat_ref = rtk_position['latitude']
                    lon_ref = rtk_position['longitude']
                    alt_ref = rtk_position.get('altitude', 0)
                    rtk_logger.set_reference(lat_ref, lon_ref, alt_ref)
                    
                    # Set initial position pada EKF
                    ekf_combined.state[0] = 0
                    ekf_combined.state[1] = 0
                    ekf_imu_only.state[0] = 0
                    ekf_imu_only.state[1] = 0
                    
                    gps_initialized = True
                    print(f"Reference set from RTK: {lat_ref:.6f}, {lon_ref:.6f} ({rtk_position.get('status', 'Unknown')})")
                
                # Fallback ke GPS biasa jika RTK belum tersedia
                elif gps_valid:
                    lat_ref = gps_coords['latitude']
                    lon_ref = gps_coords['longitude']
                    alt_ref = gps_coords.get('altitude', 0)
                    rtk_logger.set_reference(lat_ref, lon_ref, alt_ref)
                    
                    ekf_combined.state[0] = 0
                    ekf_combined.state[1] = 0
                    ekf_imu_only.state[0] = 0
                    ekf_imu_only.state[1] = 0
                    
                    gps_initialized = True
                    print(f"Reference set from standard GPS: {lat_ref:.6f}, {lon_ref:.6f}")
            
            # ===== GPS UPDATE HANYA PADA COMBINED EKF =====
            if gps_valid and gps_initialized:
                try:
                    gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                               gps_coords['longitude'],
                                               gps_coords.get('altitude', alt_ref),
                                               lat_ref, lon_ref, alt_ref)
                    
                    gps_measurement = np.array([gps_x, gps_y])
                    ekf_combined.update(gps_measurement)
                    gps_update_counter += 1
                    gps_updated = True
                except Exception as e:
                    print(f"GPS update error: {e}")
            
            # Extract final combined state
            if len(ekf_combined.state) >= 4:
                final_est_x, final_est_y, final_est_heading, final_est_velocity = ekf_combined.state[:4]
                final_est_accel = accel
            else:
                final_est_x, final_est_y = ekf_combined.state[0], ekf_combined.state[1]
                final_est_heading = 0
                final_est_velocity = 0
                final_est_accel = accel
                
            final_est_heading_deg = math.degrees(final_est_heading)
            
            # ===== RTK QUALITY TRACKING =====
            if rtk_position and not rtk_position.get('raw_data', False):
                if rtk_position.get('quality') == 1:
                    rtk_fix_count += 1
                elif rtk_position.get('quality') == 2:
                    rtk_float_count += 1
            
            # ===== ENHANCED LOGGING DENGAN RTK GROUND TRUTH =====
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                
                # RTK data untuk logging
                rtk_x, rtk_y, rtk_status = None, None, "No RTK"
                rtk_quality, rtk_satellites = 0, 0
                rtk_std_north, rtk_std_east = 0, 0
                
                if rtk_position and not rtk_position.get('raw_data', False):
                    rtk_x = rtk_position['x']
                    rtk_y = rtk_position['y']
                    rtk_status = rtk_position['status']
                    rtk_quality = rtk_position.get('quality', 0)
                    rtk_satellites = rtk_position.get('satellites', 0)
                    rtk_std_north = rtk_position.get('std_north', 0)
                    rtk_std_east = rtk_position.get('std_east', 0)
                
                log_data = [
                    current_time, dt_actual,
                    gps_x, gps_y, gps_coords['latitude'], gps_coords['longitude'],
                    accel, gyro, math.degrees(gyro),
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    rtk_x, rtk_y, rtk_status, rtk_quality, rtk_satellites,
                    rtk_std_north, rtk_std_east,
                    # Error calculation terhadap RTK ground truth
                    abs(final_est_x - rtk_x) if rtk_x is not None else None,
                    abs(final_est_y - rtk_y) if rtk_y is not None else None,
                    math.sqrt((final_est_x - rtk_x)**2 + (final_est_y - rtk_y)**2) if rtk_x is not None else None,
                    gps_updated, predict_counter, gps_update_counter
                ]
            else:
                rtk_x, rtk_y, rtk_status = None, None, "No RTK"
                if rtk_position and not rtk_position.get('raw_data', False):
                    rtk_x = rtk_position['x']
                    rtk_y = rtk_position['y']
                    rtk_status = rtk_position['status']
                    
                log_data = [
                    current_time, dt_actual,
                    None, None, None, None,
                    accel, gyro, math.degrees(gyro),
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    rtk_x, rtk_y, rtk_status, 0, 0, 0, 0,
                    abs(final_est_x - rtk_x) if rtk_x is not None else None,
                    abs(final_est_y - rtk_y) if rtk_y is not None else None,
                    None,
                    False, predict_counter, gps_update_counter
                ]
            
            log_queue.put(log_data)
            
            # ===== RTK COMPARISON LOGGING =====
            if rtk_position and not rtk_position.get('raw_data', False):
                rtk_comparison_data = [
                    current_time,
                    rtk_position['latitude'], rtk_position['longitude'], rtk_position['altitude'],
                    rtk_position['x'], rtk_position['y'],
                    rtk_position['status'], rtk_position['quality'], rtk_position['satellites'],
                    rtk_position['std_north'], rtk_position['std_east'], rtk_position['std_up'],
                    final_est_x, final_est_y,  # EKF result untuk comparison
                    gps_x if gps_valid and gps_initialized else None,
                    gps_y if gps_valid and gps_initialized else None,  # Standard GPS untuk comparison
                    abs(final_est_x - rtk_position['x']),  # EKF error
                    abs(final_est_y - rtk_position['y']),
                    abs(gps_x - rtk_position['x']) if gps_valid and gps_initialized else None,  # GPS error
                    abs(gps_y - rtk_position['y']) if gps_valid and gps_initialized else None,
                    rtk_position.get('rtcm_count', 0)
                ]
                rtk_comparison_queue.put(rtk_comparison_data)
            
            # ===== STATUS PRINT =====
            if iteration_count % 40 == 0:  # Every 2 seconds
                rtk_status_str = rtk_logger.get_rtk_status()
                
                print(f"\nt={iteration_count*dt:.1f}s | RTK Status: {rtk_status_str}")
                print(f"  Standard GPS: {gps_update_counter} updates | EKF Predicts: {predict_counter}")
                print(f"  RTK Quality: {rtk_fix_count} Fixed, {rtk_float_count} Float")
                
                if rtk_position and not rtk_position.get('raw_data', False):
                    print(f"  Positions - EKF: ({final_est_x:.2f}, {final_est_y:.2f}) | "
                          f"RTK: ({rtk_position['x']:.2f}, {rtk_position['y']:.2f})")
                    print(f"  RTK Accuracy: ±{rtk_position.get('std_north', 0):.3f}m N, "
                          f"±{rtk_position.get('std_east', 0):.3f}m E")
                    
                    # Calculate error terhadap RTK ground truth
                    error_distance = math.sqrt((final_est_x - rtk_position['x'])**2 + 
                                             (final_est_y - rtk_position['y'])**2)
                    print(f"  EKF Error vs RTK: {error_distance:.3f}m")
                
                print(f"  RTCM packets: {rtk_corrector.rtcm_count}")
            
            iteration_count += 1
            last_time = current_time
            
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
        
        # Stop handlers
        gps_handler.stop()
        imu_handler.stop()
        rtk_corrector.stop()
        
        # Stop logging threads
        log_queue.put("STOP")
        rtk_comparison_queue.put("STOP")
        imu_predict_queue.put("STOP")
        
        # Wait for threads to finish
        for thread in [log_thread, rtk_comparison_thread, imu_predict_thread]:
            if thread.is_alive():
                thread.join(timeout=2)
        
        print(f"✅ Data logged dengan RTK ground truth:")
        print(f"   Main log: {log_filename}")
        print(f"   RTK comparison: {rtk_comparison_filename}")
        print(f"   IMU predict: {imu_predict_filename}")
        print("✅ RTK telah digunakan sebagai ground truth untuk evaluasi akurasi!")

# ===== ENHANCED LOGGING FUNCTIONS =====

def enhanced_logging_func(q: Queue, filename: str):
    """Enhanced combined logging dengan RTK ground truth"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
            'rtk_x', 'rtk_y', 'rtk_status', 'rtk_quality', 'rtk_satellites',
            'rtk_std_north', 'rtk_std_east',
            'error_x_vs_rtk', 'error_y_vs_rtk', 'error_distance_vs_rtk',
            'gps_updated', 'predict_count', 'gps_update_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def rtk_comparison_logging_func(q: Queue, filename: str):
    """Specialized logging untuk RTK ground truth comparison"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'rtk_lat', 'rtk_lon', 'rtk_alt',
            'rtk_x', 'rtk_y', 'rtk_status', 'rtk_quality', 'rtk_satellites',
            'rtk_std_north', 'rtk_std_east', 'rtk_std_up',
            'ekf_x', 'ekf_y', 'gps_x', 'gps_y',
            'ekf_error_x', 'ekf_error_y', 'gps_error_x', 'gps_error_y',
            'rtcm_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def imu_predict_logging_func(q: Queue, filename: str):
    """IMU predict logging dengan RTK comparison"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'predict_x', 'predict_y', 'predict_heading_deg', 
            'predict_velocity', 'predict_accel', 
            'rtk_x', 'rtk_y', 'rtk_status',
            'error_x_vs_rtk', 'error_y_vs_rtk',
            'predict_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def main():
    """Main function dengan RTK ground truth option"""
    print("GPS-IMU Sensor Fusion dengan RTK Ground Truth")
    print("RTK digunakan sebagai referensi akurasi, tidak dalam EKF")
    print()
    print("Pilihan:")
    print("1. Run sensor fusion dengan RTK ground truth")
    print("2. Test RTK system only")
    print("3. Run standard fusion tanpa RTK")
    
    choice = input("Masukkan pilihan (1-3): ").strip()
    
    if choice == '1':
        main_sensor_fusion_with_rtk_ground_truth()
    elif choice == '2':
        test_rtk_only()
    elif choice == '3':
        # Import from original code
        from main_original import main_sensor_fusion_with_filters
        main_sensor_fusion_with_filters()
    else:
        print("Pilihan tidak valid")

def test_rtk_only():
    """Test RTK system saja"""
    print("=== Testing RTK System Only ===")
    
    rtk_corrector = RTKLIBGPSCorrector(gps_port='/dev/ttyUSB0', gps_baudrate=115200)
    rtk_logger = RTKGroundTruthLogger(rtk_corrector)
    
    # Start RTK system
    rtk_gps_thread = threading.Thread(target=rtk_corrector.read_gps_data)
    rtk_gps_thread.daemon = True
    rtk_gps_thread.start()
    
    # NTRIP Configuration untuk test
    ntrip_config = {
        'user': 'toor:toor123456',
        'caster': 'nrtk.big.go.id',
        'port': 2001,
        'mountpoint': '/vrs-rtcm3',
        'lat': -7.2839114,
        'lon': 112.7961259,
        'height': 10.0,
        'verbose': False,
        'out': BytesIO()
    }
    
    # Modified NTRIP receive untuk test
    def test_receive_data(client_self):
        lastGGATime = time.time()
        try:
            while True:
                current_time = time.time()
                
                if rtk_corrector.raw_gps_data:
                    with rtk_corrector.gps_lock:
                        client_self.setPosition(
                            rtk_corrector.raw_gps_data['latitude'],
                            rtk_corrector.raw_gps_data['longitude']
                        )
                        client_self.height = rtk_corrector.raw_gps_data['altitude']
                
                if current_time - lastGGATime > 10:
                    client_self.socket.send(client_self.getGGAString().encode())
                    lastGGATime = current_time
                
                client_self.socket.settimeout(1.0)
                try:
                    data = client_self.socket.recv(client_self.buffer)
                    if not data:
                        break
                    rtk_corrector.on_rtcm_data(data)
                except Exception:
                    continue
        except KeyboardInterrupt:
            print("NTRIP stopped")
        finally:
            if client_self.socket:
                client_self.socket.close()
    
    print("Starting NTRIP client...")
    NtripClient.receiveData = test_receive_data
    ntrip_client = NtripClient(**ntrip_config)
    ntrip_thread = threading.Thread(target=ntrip_client.readLoop)
    ntrip_thread.daemon = True
    ntrip_thread.start()
    
    print("Monitoring RTK system for 60 seconds...")
    print("Time\tRaw GPS\t\t\tRTK Solution\t\t\tStatus\t\tRTCM")
    print("-" * 100)
    
    try:
        start_time = time.time()
        rtk_logger.set_reference(-7.2839114, 112.7961259, 10.0)  # Set manual reference
        
        for i in range(600):  # 60 seconds at 10Hz
            elapsed = time.time() - start_time
            
            # Raw GPS data
            raw_gps_str = "No GPS"
            if rtk_corrector.raw_gps_data:
                with rtk_corrector.gps_lock:
                    raw_data = rtk_corrector.raw_gps_data
                    raw_gps_str = f"{raw_data['latitude']:.6f},{raw_data['longitude']:.6f}"
            
            # RTK solution
            rtk_position = rtk_logger.get_rtk_position()
            rtk_str = "No RTK"
            rtk_status = "Processing"
            
            if rtk_position and not rtk_position.get('raw_data', False):
                rtk_str = f"{rtk_position['latitude']:.6f},{rtk_position['longitude']:.6f}"
                rtk_status = f"{rtk_position['status']} (Q:{rtk_position['quality']})"
            elif rtk_position and rtk_position.get('raw_data', False):
                rtk_str = f"Init: {rtk_position['latitude']:.6f},{rtk_position['longitude']:.6f}"
                rtk_status = f"{rtk_position.get('status', 'Unknown')}"
            
            if i % 10 == 0:  # Print every second
                print(f"{elapsed:.1f}s\t{raw_gps_str:<20}\t{rtk_str:<20}\t{rtk_status:<15}\t{rtk_corrector.rtcm_count}")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nRTK test stopped by user")
    finally:
        rtk_corrector.stop()
        print(f"Final RTCM count: {rtk_corrector.rtcm_count}")
        print("RTK test completed")

if __name__ == "__main__":
    main()