import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock
from queue import Queue
import serial
from io import BytesIO
from ntripclient import NtripClient
import pynmea2

# Import custom modules
from gpsimuhandler import GPSHandler, FilteredIMUHandler, create_imu_handler
from ekfnparam3 import EKFSensorFusion
from helper import latlon_to_xy, is_gps_valid

class RTKGroundTruthHandler:
    """RTK Handler sebagai ground truth dengan NTRIP streaming"""
    
    def __init__(self, rtk_port='/dev/ttyUSB1', rtk_baudrate=115200):
        self.rtk_port = rtk_port
        self.rtk_baudrate = rtk_baudrate
        self.rtk_serial = None
        
        # RTK Status
        self.rtcm_count = 0
        self.last_rtcm_time = 0
        self.current_rtk_position = {}
        self.position_history = []
        
        # Buffer for proper NMEA parsing
        self.nmea_buffer = ""
        
        # Threading
        self.running = False
        self.data_lock = Lock()
        
        # Stats
        self.stats = {
            'read_count': 0,
            'error_count': 0,
            'rtk_fix_count': 0,
            'last_update_time': 0
        }
        
        print(f"🎯 RTK Ground Truth Handler initialized")
        print(f"RTK Port: {rtk_port} @ {rtk_baudrate}")
    
    def connect_rtk(self):
        """Connect to RTK receiver"""
        try:
            self.rtk_serial = serial.Serial(
                port=self.rtk_port,
                baudrate=self.rtk_baudrate,
                timeout=0.1
            )
            print(f"✓ RTK GPS connected")
            return True
        except Exception as e:
            print(f"❌ RTK GPS connection failed: {e}")
            return False
    
    def start(self):
        """Start RTK reading thread"""
        self.running = True
        self.rtk_thread = Thread(target=self.read_rtk_data)
        self.rtk_thread.daemon = True
        self.rtk_thread.start()
        return True
    
    def stop(self):
        """Stop RTK handler"""
        self.running = False
        if hasattr(self, 'rtk_thread') and self.rtk_thread.is_alive():
            self.rtk_thread.join(timeout=2)
        if self.rtk_serial and self.rtk_serial.is_open:
            self.rtk_serial.close()
    
    def read_rtk_data(self):
        """Read RTK data and monitor status"""
        if not self.connect_rtk():
            return
        
        print("📡 Reading RTK data for ground truth...")
        
        while self.running:
            try:
                if self.rtk_serial.in_waiting > 0:
                    data = self.rtk_serial.read(self.rtk_serial.in_waiting)
                    
                    # Process as text for NMEA
                    try:
                        text_data = data.decode('ascii', errors='ignore')
                        self.nmea_buffer += text_data
                        self.process_nmea_buffer()
                        self.stats['read_count'] += 1
                    except Exception as e:
                        self.stats['error_count'] += 1
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"❌ RTK read error: {e}")
                self.stats['error_count'] += 1
                time.sleep(1)
    
    def process_nmea_buffer(self):
        """Process NMEA buffer to extract complete sentences"""
        lines = self.nmea_buffer.split('\n')
        self.nmea_buffer = lines[-1]  # Keep incomplete line
        
        for line in lines[:-1]:
            line = line.strip()
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                if '*' in line:  # Complete sentence
                    self.process_gga_sentence(line)
    
    def process_gga_sentence(self, sentence):
        """Process GGA sentence and extract RTK info"""
        try:
            # Try pynmea2 first
            msg = pynmea2.parse(sentence)
            
            if msg.latitude and msg.longitude:
                with self.data_lock:
                    self.current_rtk_position = {
                        'timestamp': time.time(),
                        'time': str(msg.timestamp) if msg.timestamp else '',
                        'latitude': float(msg.latitude),
                        'longitude': float(msg.longitude),
                        'altitude': float(msg.altitude) if msg.altitude else 0.0,
                        'quality': int(msg.gps_qual) if msg.gps_qual else 0,
                        'satellites': int(msg.num_sats) if msg.num_sats else 0,
                        'hdop': float(msg.horizontal_dil) if msg.horizontal_dil else 99.0,
                        'rtcm_age': float(msg.age_gps_data) if hasattr(msg, 'age_gps_data') and msg.age_gps_data and msg.age_gps_data != '' else 0,
                        'diff_station': msg.diff_ref_station_id if hasattr(msg, 'diff_ref_station_id') and msg.diff_ref_station_id else '',
                        'raw_sentence': sentence
                    }
                    
                    # Count RTK fixes
                    if self.current_rtk_position['quality'] >= 4:  # RTK Fixed or Float
                        self.stats['rtk_fix_count'] += 1
                    
                    self.stats['last_update_time'] = time.time()
                    
                    self.position_history.append(self.current_rtk_position.copy())
                    if len(self.position_history) > 10:
                        self.position_history.pop(0)
                        
        except Exception as e:
            # Try manual parsing
            self.manual_parse_gga(sentence)
    
    def manual_parse_gga(self, sentence):
        """Manual parsing for problematic GGA sentences"""
        try:
            sentence_part = sentence.split('*')[0]
            fields = sentence_part.split(',')
            
            if len(fields) >= 15:
                with self.data_lock:
                    self.current_rtk_position = {
                        'timestamp': time.time(),
                        'time': fields[1],
                        'latitude': self.parse_coordinate(fields[2], fields[3]),
                        'longitude': self.parse_coordinate(fields[4], fields[5]),
                        'altitude': float(fields[9]) if fields[9] else 0.0,
                        'quality': int(fields[6]) if fields[6] else 0,
                        'satellites': int(fields[7]) if fields[7] else 0,
                        'hdop': float(fields[8]) if fields[8] else 99.0,
                        'rtcm_age': float(fields[13]) if fields[13] else 0,
                        'diff_station': fields[14] if len(fields) > 14 and fields[14] else '',
                        'raw_sentence': sentence
                    }
                    
                    if self.current_rtk_position['quality'] >= 4:
                        self.stats['rtk_fix_count'] += 1
                    
                    self.stats['last_update_time'] = time.time()
                    
                    self.position_history.append(self.current_rtk_position.copy())
                    if len(self.position_history) > 10:
                        self.position_history.pop(0)
                        
        except Exception as e:
            pass  # Silent fail for manual parsing
    
    def parse_coordinate(self, coord_str, direction):
        """Parse coordinate from NMEA format"""
        try:
            if not coord_str or coord_str == '':
                return 0.0
            
            degrees = int(float(coord_str) // 100)
            minutes = float(coord_str) % 100
            decimal = degrees + minutes / 60.0
            
            if direction in ['S', 'W']:
                decimal = -decimal
                
            return decimal
        except:
            return 0.0
    
    def send_rtcm_to_rtk(self, rtcm_data):
        """Send RTCM data to RTK GPS"""
        if not self.rtk_serial or not self.rtk_serial.is_open:
            return False
        
        try:
            self.rtk_serial.write(rtcm_data)
            self.rtk_serial.flush()
            
            with self.data_lock:
                self.rtcm_count += 1
                self.last_rtcm_time = time.time()
            
            return True
        except Exception as e:
            print(f"❌ RTK RTCM send error: {e}")
            return False
    
    def get_rtk_coords(self):
        """Get current RTK coordinates"""
        with self.data_lock:
            if self.current_rtk_position and time.time() - self.stats['last_update_time'] < 5.0:
                return self.current_rtk_position.copy()
            return None
    
    def get_stats(self):
        """Get RTK handler statistics"""
        with self.data_lock:
            stats = self.stats.copy()
            if self.current_rtk_position:
                stats['current_quality'] = self.current_rtk_position.get('quality', 0)
                stats['rtk_active'] = self.current_rtk_position.get('quality', 0) >= 4
                stats['rtcm_count'] = self.rtcm_count
                stats['last_rtcm_age'] = time.time() - self.last_rtcm_time if self.last_rtcm_time > 0 else 999
            return stats

def main_ekf_with_rtk_ground_truth():
    """
    Main function: EKF Sensor Fusion + IMU Predict + RTK Ground Truth
    Opsi 7: Lengkap dengan perbandingan akurasi terhadap RTK
    """
    
    print("=== EKF SENSOR FUSION WITH RTK GROUND TRUTH ===")
    print("🎯 RTK sebagai ground truth untuk validasi akurasi")
    print("📊 Membandingkan: EKF vs IMU-only vs RTK")
    
    # ===== CONFIGURATION =====
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
    
    # NTRIP configuration for RTK
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
    
    print(f"Filter config: Accel={filter_config['accel_filter_config']['cutoff']}Hz, "
          f"Gyro={filter_config['gyro_filter_config']['cutoff']}Hz")
    
    # ===== INITIALIZE HARDWARE =====
    print("\n🔧 Initializing hardware...")
    
    # GPS biasa untuk EKF
    print("Initializing standard GPS for EKF...")
    gps_handler = GPSHandler()  # /dev/ttyUSB0
    time.sleep(2)
    
    # RTK GPS untuk ground truth
    print("Initializing RTK GPS for ground truth...")
    rtk_handler = RTKGroundTruthHandler(rtk_port='/dev/ttyUSB1')  # Port berbeda
    time.sleep(2)
    
    # IMU dengan filtering
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(**filter_config)
    time.sleep(2)
    
    # Validate IMU
    test_accel, test_gyro = imu_handler.get_data()
    print(f"IMU test: accel={test_accel:.3f}, gyro={math.degrees(test_gyro):.1f}°/s")
    
    # ===== INITIALIZE EKF =====
    print("Initializing EKF instances...")
    dt = 0.05  # 20Hz
    
    ekf_combined = EKFSensorFusion(dt=dt)  # GPS+IMU fusion
    ekf_imu_only = EKFSensorFusion(dt=dt)  # IMU predict only
    
    # ===== SETUP LOGGING =====
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    log_queues = {
        'combined': Queue(),
        'imu_predict': Queue(),
        'rtk_ground_truth': Queue(),
        'accuracy_comparison': Queue()
    }
    
    log_filenames = {
        'combined': f"ekf_combined_rtk_{timestamp}.csv",
        'imu_predict': f"imu_predict_rtk_{timestamp}.csv", 
        'rtk_ground_truth': f"rtk_ground_truth_{timestamp}.csv",
        'accuracy_comparison': f"accuracy_vs_rtk_{timestamp}.csv"
    }
    
    # Start logging threads
    logging_threads = []
    for log_type, queue in log_queues.items():
        thread = Thread(target=globals()[f'{log_type}_logging_func'], 
                       args=(queue, log_filenames[log_type]))
        thread.daemon = True
        thread.start()
        logging_threads.append(thread)
    
    # ===== START RTK HANDLER =====
    print("Starting RTK ground truth handler...")
    rtk_handler.start()
    
    # ===== SETUP NTRIP FOR RTK =====
    def setup_rtk_ntrip():
        """Setup NTRIP client untuk RTK corrections"""
        def new_receive_data(self):
            lastGGATime = time.time()
            
            try:
                while rtk_handler.running:
                    current_time = time.time()
                    
                    # Send GGA to NTRIP
                    if current_time - lastGGATime > 10:
                        rtk_coords = rtk_handler.get_rtk_coords()
                        if rtk_coords:
                            self.setPosition(rtk_coords['latitude'], rtk_coords['longitude'])
                            self.height = rtk_coords['altitude']
                        else:
                            self.setPosition(ntrip_config['lat'], ntrip_config['lon'])
                            self.height = ntrip_config['height']
                        
                        self.socket.send(self.getGGAString().encode())
                        lastGGATime = current_time
                    
                    self.socket.settimeout(1.0)
                    try:
                        data = self.socket.recv(self.buffer)
                        if not data:
                            break
                        
                        # Forward RTCM to RTK GPS
                        rtk_handler.send_rtcm_to_rtk(data)
                        
                    except Exception:
                        continue
                        
            except KeyboardInterrupt:
                print("RTK NTRIP stopped")
            finally:
                if self.socket:
                    self.socket.close()
        
        # Patch NtripClient
        NtripClient.receiveData = new_receive_data
        
        # Start NTRIP client
        client = NtripClient(**ntrip_config)
        ntrip_thread = Thread(target=client.readLoop)
        ntrip_thread.daemon = True
        ntrip_thread.start()
        
        return client, ntrip_thread
    
    print("Setting up RTK NTRIP corrections...")
    try:
        rtk_client, rtk_ntrip_thread = setup_rtk_ntrip()
        print("✓ RTK NTRIP client started")
    except Exception as e:
        print(f"⚠️ RTK NTRIP setup failed: {e}")
        print("Continuing without NTRIP corrections...")
    
    # ===== MAIN LOOP VARIABLES =====
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    rtk_initialized = False
    last_time = time.time()
    rtk_x, rtk_y = 0, 0  # Initialize RTK coordinates
    
    counters = {
        'iteration': 0,
        'predict': 0,
        'gps_update': 0,
        'rtk_update': 0
    }
    
    print(f"\n🚀 Starting integrated system...")
    print(f"📁 Logs will be saved as:")
    for log_type, filename in log_filenames.items():
        print(f"   {log_type}: {filename}")
    
    try:
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
            except Exception as e:
                print(f"IMU read error: {e}")
                accel_filtered = gyro_filtered = 0
                accel_raw = gyro_raw = 0
                filter_stats = {'accel_noise_reduction': 0, 'gyro_noise_reduction': 0}
            
            # ===== EKF PREDICT (BOTH INSTANCES) =====
            try:
                # Combined EKF (akan di-update dengan GPS)
                ekf_combined.predict(imu_accel=accel_filtered, imu_omega=gyro_filtered, dt=dt_actual)
                
                # IMU-only EKF (TIDAK akan di-update GPS)
                ekf_imu_only.predict(imu_accel=accel_filtered, imu_omega=gyro_filtered, dt=dt_actual)
                
                counters['predict'] += 1
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                last_time = current_time
                continue
            
            # ===== READ STANDARD GPS =====
            gps_coords = gps_handler.get_coords()
            gps_valid = is_gps_valid(gps_coords)
            gps_updated = False
            
            # ===== READ RTK GPS =====
            rtk_coords = rtk_handler.get_rtk_coords()
            rtk_valid = rtk_coords is not None and rtk_coords.get('quality', 0) >= 4  # RTK Fixed/Float
            rtk_updated = rtk_coords is not None
            
            # ===== INITIALIZE REFERENCE COORDINATES =====
            if not gps_initialized and gps_valid:
                lat_ref = gps_coords['latitude']
                lon_ref = gps_coords['longitude']
                alt_ref = gps_coords.get('altitude', 0)
                
                # Set initial position untuk EKF
                x_gps, y_gps = latlon_to_xy(lat_ref, lon_ref, alt_ref, lat_ref, lon_ref, alt_ref)
                ekf_combined.state[0] = x_gps  # Should be 0
                ekf_combined.state[1] = y_gps  # Should be 0
                ekf_imu_only.state[0] = x_gps
                ekf_imu_only.state[1] = y_gps
                
                gps_initialized = True
                print(f"📍 GPS reference set: {lat_ref:.6f}, {lon_ref:.6f}")
            
            # Atau gunakan RTK sebagai reference jika lebih dulu available
            if not rtk_initialized and rtk_valid and not gps_initialized:
                lat_ref = rtk_coords['latitude']
                lon_ref = rtk_coords['longitude']
                alt_ref = rtk_coords.get('altitude', 0)
                
                # Set initial position
                x_rtk, y_rtk = latlon_to_xy(lat_ref, lon_ref, alt_ref, lat_ref, lon_ref, alt_ref)
                ekf_combined.state[0] = x_rtk
                ekf_combined.state[1] = y_rtk
                ekf_imu_only.state[0] = x_rtk
                ekf_imu_only.state[1] = y_rtk
                
                rtk_initialized = True
                gps_initialized = True  # Use RTK as reference
                print(f"🎯 RTK reference set: {lat_ref:.6f}, {lon_ref:.6f}")
            
            # ===== GPS UPDATE (HANYA COMBINED EKF) =====
            if gps_valid and gps_initialized:
                try:
                    gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                               gps_coords['longitude'],
                                               gps_coords.get('altitude', alt_ref),
                                               lat_ref, lon_ref, alt_ref)
                    
                    gps_measurement = np.array([gps_x, gps_y])
                    ekf_combined.update(gps_measurement)
                    counters['gps_update'] += 1
                    gps_updated = True
                except Exception as e:
                    print(f"GPS update error: {e}")
            
            # ===== EXTRACT STATES =====
            # Combined EKF state
            if len(ekf_combined.state) >= 4:
                combined_x, combined_y, combined_heading, combined_velocity = ekf_combined.state[:4]
            else:
                combined_x, combined_y = ekf_combined.state[0], ekf_combined.state[1]
                combined_heading = combined_velocity = 0
            
            # IMU-only state
            if len(ekf_imu_only.state) >= 4:
                imu_only_x, imu_only_y, imu_only_heading, imu_only_velocity = ekf_imu_only.state[:4]
            else:
                imu_only_x, imu_only_y = ekf_imu_only.state[0], ekf_imu_only.state[1]
                imu_only_heading = imu_only_velocity = 0
            
            combined_heading_deg = math.degrees(combined_heading)
            imu_only_heading_deg = math.degrees(imu_only_heading)
            
            # ===== CALCULATE ACCURACY VS RTK =====
            errors = {
                'ekf_vs_rtk_distance': None,
                'imu_vs_rtk_distance': None,
                'gps_vs_rtk_distance': None,
                'ekf_vs_rtk_heading': None,
                'imu_vs_rtk_heading': None
            }
            
            if rtk_valid and gps_initialized:
                # RTK position in local coordinates
                rtk_x, rtk_y = latlon_to_xy(rtk_coords['latitude'], 
                                           rtk_coords['longitude'],
                                           rtk_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                
                # Distance errors
                errors['ekf_vs_rtk_distance'] = math.sqrt((combined_x - rtk_x)**2 + (combined_y - rtk_y)**2)
                errors['imu_vs_rtk_distance'] = math.sqrt((imu_only_x - rtk_x)**2 + (imu_only_y - rtk_y)**2)
                
                if gps_valid:
                    gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                               gps_coords['longitude'],
                                               gps_coords.get('altitude', alt_ref),
                                               lat_ref, lon_ref, alt_ref)
                    errors['gps_vs_rtk_distance'] = math.sqrt((gps_x - rtk_x)**2 + (gps_y - rtk_y)**2)
                
                counters['rtk_update'] += 1
            
            # ===== LOGGING =====
            
            # Combined EKF logging
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                combined_log_data = [
                    current_time, dt_actual,
                    gps_x, gps_y, gps_coords['latitude'], gps_coords['longitude'],
                    accel_filtered, gyro_filtered, math.degrees(gyro_filtered),
                    accel_raw, gyro_raw, math.degrees(gyro_raw),
                    combined_x, combined_y, combined_heading_deg, combined_velocity,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    gps_updated, counters['predict'], counters['gps_update']
                ]
            else:
                combined_log_data = [
                    current_time, dt_actual,
                    None, None, None, None,
                    accel_filtered, gyro_filtered, math.degrees(gyro_filtered),
                    accel_raw, gyro_raw, math.degrees(gyro_raw),
                    combined_x, combined_y, combined_heading_deg, combined_velocity,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    False, counters['predict'], counters['gps_update']
                ]
            
            log_queues['combined'].put(combined_log_data)
            
            # IMU predict logging
            imu_predict_log_data = [
                current_time, dt_actual,
                accel_filtered, gyro_filtered, math.degrees(gyro_filtered),
                accel_raw, gyro_raw, math.degrees(gyro_raw),
                imu_only_x, imu_only_y, imu_only_heading_deg, imu_only_velocity,
                filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                counters['predict']
            ]
            log_queues['imu_predict'].put(imu_predict_log_data)
            
            # RTK ground truth logging
            if rtk_updated:
                rtk_x, rtk_y = latlon_to_xy(rtk_coords['latitude'], 
                                           rtk_coords['longitude'],
                                           rtk_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref) if gps_initialized else (0, 0)
                
                rtk_log_data = [
                    current_time,
                    rtk_coords['latitude'], rtk_coords['longitude'], rtk_coords.get('altitude', 0),
                    rtk_x, rtk_y,
                    rtk_coords['quality'], rtk_coords['satellites'], rtk_coords['hdop'],
                    rtk_coords.get('rtcm_age', 0), rtk_coords.get('diff_station', ''),
                    rtk_valid, counters['rtk_update']
                ]
                log_queues['rtk_ground_truth'].put(rtk_log_data)
            
            # Accuracy comparison logging
            if any(error is not None for error in errors.values()):
                accuracy_log_data = [
                    current_time,
                    errors['ekf_vs_rtk_distance'], errors['imu_vs_rtk_distance'], errors['gps_vs_rtk_distance'],
                    errors['ekf_vs_rtk_heading'], errors['imu_vs_rtk_heading'],
                    rtk_valid, gps_valid,
                    counters['iteration']
                ]
                log_queues['accuracy_comparison'].put(accuracy_log_data)
            
            # ===== STATUS DISPLAY =====
            if counters['iteration'] % 40 == 0:  # Every 2 seconds
                gps_stats = gps_handler.get_stats()
                rtk_stats = rtk_handler.get_stats()
                imu_stats = imu_handler.get_stats()
                
                print(f"\n⏱️  t={counters['iteration']*dt:.1f}s | Predicts: {counters['predict']} | "
                      f"GPS updates: {counters['gps_update']} | RTK updates: {counters['rtk_update']}")
                
                print(f"📊 Stats - GPS: {gps_stats['read_count']}/{gps_stats.get('error_count', 0)} | "
                      f"RTK: {rtk_stats['read_count']}/{rtk_stats['error_count']} | "
                      f"IMU: {imu_stats['read_count']}/{imu_stats['error_count']}")
                
                print(f"📍 Positions:")
                print(f"   🎯 RTK (truth): ({rtk_x:.3f}, {rtk_y:.3f}) | Quality: {rtk_coords.get('quality', 0) if rtk_coords else 'N/A'}")
                print(f"   🔄 EKF (GPS+IMU): ({combined_x:.3f}, {combined_y:.3f})")
                print(f"   📐 IMU-only: ({imu_only_x:.3f}, {imu_only_y:.3f})")
                
                if errors['ekf_vs_rtk_distance'] is not None:
                    print(f"📏 Accuracy vs RTK:")
                    print(f"   EKF error: {errors['ekf_vs_rtk_distance']:.3f}m")
                    print(f"   IMU error: {errors['imu_vs_rtk_distance']:.3f}m")
                    if errors['gps_vs_rtk_distance'] is not None:
                        print(f"   GPS error: {errors['gps_vs_rtk_distance']:.3f}m")
                
                print(f"🔧 Filter effectiveness: A={filter_stats['accel_noise_reduction']:.1%}, "
                      f"G={filter_stats['gyro_noise_reduction']:.1%}")
                
                # Quality warnings
                if rtk_coords and rtk_coords.get('quality', 0) < 4:
                    print(f"   ⚠️ RTK not in Fixed/Float mode (Quality: {rtk_coords.get('quality', 0)})")
                
                if not gps_valid:
                    print(f"   ⚠️ Standard GPS invalid")
                
                # Accuracy warnings
                if errors['ekf_vs_rtk_distance'] is not None:
                    if errors['ekf_vs_rtk_distance'] > 5.0:
                        print(f"   ⚠️ High EKF error vs RTK!")
                    if errors['imu_vs_rtk_distance'] > 20.0:
                        print(f"   ⚠️ IMU drift detected!")
            
            counters['iteration'] += 1
            last_time = current_time
            
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    except Exception as e:
        print(f"💥 Main loop error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🧹 Cleaning up...")
        
        # Stop handlers
        gps_handler.stop()
        rtk_handler.stop()
        imu_handler.stop()
        
        # Stop logging threads
        for queue in log_queues.values():
            queue.put("STOP")
        
        # Wait for threads to finish
        for thread in logging_threads:
            if thread.is_alive():
                thread.join(timeout=2)
        
        print(f"✅ Data logged to:")
        for log_type, filename in log_filenames.items():
            print(f"   📄 {log_type}: {filename}")
        
        print("✅ RTK-EKF comparison completed!")
        print("📊 Analyze logs to compare EKF vs IMU-only vs RTK ground truth accuracy")

# ===== LOGGING FUNCTIONS =====

def combined_logging_func(q: Queue, filename: str):
    """Combined EKF logging dengan RTK comparison"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'ekf_x', 'ekf_y', 'ekf_heading_deg', 'ekf_velocity',
            'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'gps_updated', 'predict_count', 'gps_update_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def imu_predict_logging_func(q: Queue, filename: str):
    """IMU predict only logging"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'imu_only_x', 'imu_only_y', 'imu_only_heading_deg', 'imu_only_velocity',
            'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'predict_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def rtk_ground_truth_logging_func(q: Queue, filename: str):
    """RTK ground truth logging"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'rtk_lat', 'rtk_lon', 'rtk_alt', 'rtk_x', 'rtk_y',
            'rtk_quality', 'rtk_satellites', 'rtk_hdop', 'rtcm_age', 'diff_station',
            'rtk_valid', 'rtk_update_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def accuracy_comparison_logging_func(q: Queue, filename: str):
    """Accuracy comparison logging vs RTK ground truth"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'ekf_vs_rtk_distance_m', 'imu_vs_rtk_distance_m', 'gps_vs_rtk_distance_m',
            'ekf_vs_rtk_heading_deg', 'imu_vs_rtk_heading_deg',
            'rtk_valid', 'gps_valid', 'iteration'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

# ===== UPDATED MAIN FUNCTION WITH OPTION 7 =====

def main():
    """Main function dengan pilihan testing dan konfigurasi + RTK option"""
    print("GPS-IMU Sensor Fusion dengan Low-Pass Filtering + RTK Ground Truth")
    print("Mengurangi noise getaran mesin untuk navigasi yang lebih akurat")
    print("🎯 RTK sebagai ground truth untuk validasi akurasi")
    print()
    print("Pilihan:")
    print("1. Run main sensor fusion dengan filtering")
    print("2. Test filter effectiveness real-time")
    print("3. Test berbagai konfigurasi filter")
    print("4. Test IMU predict only (dengan filtering)")
    print("5. Test GPS only")
    print("6. Quick filter demo")
    print("🆕 7. EKF + IMU Predict + RTK Ground Truth (COMPLETE COMPARISON)")
    
    choice = input("Masukkan pilihan (1-7): ").strip()
    
    if choice == '1':
        main_sensor_fusion_with_filters()
    elif choice == '2':
        test_filter_effectiveness()
    elif choice == '3':
        test_different_filter_settings()
    elif choice == '4':
        test_imu_predict_only_filtered()
    elif choice == '5':
        test_gps_only()
    elif choice == '6':
        quick_filter_demo()
    elif choice == '7':
        print("🚀 Starting complete RTK-EKF comparison system...")
        main_ekf_with_rtk_ground_truth()
    else:
        print("Pilihan tidak valid")

# ===== EXISTING FUNCTIONS (unchanged) =====

def main_sensor_fusion_with_filters():
    """
    Main sensor fusion dengan low-pass filtering untuk mengurangi noise getaran mesin
    Menggunakan dua EKF instance terpisah untuk logging yang benar
    """
    
    print("=== SENSOR FUSION WITH LOW-PASS FILTERING ===")
    print("Reducing engine vibration noise with adaptive filters")
    
    # ===== FILTER CONFIGURATION =====
    # Sesuaikan dengan jenis kendaraan/aplikasi Anda
    filter_config = {
        'application_type': 'vehicle',  # 'vehicle', 'drone', 'walking_robot'
        'accel_filter_config': {
            'cutoff': 3.0,        # Hz - Allow vehicle dynamics, filter engine vibration (>10Hz)
            'type': 'butterworth', # 'butterworth', 'exponential', 'moving_average'
            'adaptive': False      # True untuk adaptive filtering
        },
        'gyro_filter_config': {
            'cutoff': 5.0,         # Hz - Allow turning dynamics
            'type': 'butterworth',
            'adaptive': False
        }
    }
    
    # Option untuk testing different filter settings
    print("Filter Configuration:")
    print(f"  Application type: {filter_config['application_type']}")
    print(f"  Accel filter: {filter_config['accel_filter_config']['cutoff']} Hz {filter_config['accel_filter_config']['type']}")
    print(f"  Gyro filter: {filter_config['gyro_filter_config']['cutoff']} Hz {filter_config['gyro_filter_config']['type']}")
    print(f"  Adaptive filtering: {filter_config['accel_filter_config']['adaptive']}")
    
    # Initialize hardware dengan filtering
    print("\nInitializing GPS...")
    gps_handler = GPSHandler()
    time.sleep(2)
    
    print("Initializing IMU with filtering...")
    imu_handler = FilteredIMUHandler(**filter_config)
    time.sleep(2)
    
    # Validate IMU is working
    test_accel, test_gyro = imu_handler.get_data()
    print(f"IMU test read: accel={test_accel:.3f}, gyro={math.degrees(test_gyro):.1f}°/s")
    
    print("Initializing EKF...")
    dt = 0.05  # 20Hz
    
    # ✅ DUA EKF INSTANCE TERPISAH
    ekf_combined = EKFSensorFusion(dt=dt)  # Untuk combined fusion
    ekf_imu_only = EKFSensorFusion(dt=dt)  # Untuk IMU-only predict
    
    # Setup logging dengan filter info
    log_queue = Queue()  # Combined logging
    imu_predict_queue = Queue()  # IMU predict only logging
    filter_stats_queue = Queue()  # Filter effectiveness logging
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f"sensor_fusion_filtered_{timestamp}.csv"
    imu_predict_filename = f"imu_predict_filtered_{timestamp}.csv"
    filter_stats_filename = f"filter_stats_{timestamp}.csv"
    
    # Logging threads
    log_thread = Thread(target=combined_logging_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    imu_predict_thread = Thread(target=imu_predict_logging_func, args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    filter_stats_thread = Thread(target=filter_stats_logging_func, args=(filter_stats_queue, filter_stats_filename))
    filter_stats_thread.daemon = True
    filter_stats_thread.start()
    
    # Variables
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    predict_counter = 0
    
    print(f"\nStarting filtered sensor fusion...")
    print(f"Combined log: {log_filename}")
    print(f"IMU predict log: {imu_predict_filename}")
    print(f"Filter stats log: {filter_stats_filename}")
    print("Logs will show effectiveness of vibration filtering")
    
    try:
        iteration_count = 0
        filter_stats_log_counter = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # ===== READ FILTERED IMU DATA =====
            try:
                # Get both filtered and raw data untuk comparison
                accel_filtered, gyro_filtered, accel_raw, gyro_raw, filter_stats = imu_handler.get_data(
                    return_raw=True, return_stats=True
                )
                
                # Use filtered data for EKF
                accel = accel_filtered
                gyro = gyro_filtered
                
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
                accel_raw, gyro_raw = 0, 0
                filter_stats = {'accel_noise_reduction': 0, 'gyro_noise_reduction': 0}
            
            # ===== EKF PREDICT PADA KEDUA INSTANCE =====
            try:
                # Combined EKF predict (akan di-update GPS nanti)
                ekf_combined.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                # IMU-only EKF predict (TIDAK AKAN di-update GPS!)
                ekf_imu_only.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                predict_counter += 1
                
                # Extract pure IMU predict dari EKF terpisah
                pure_predict_state = ekf_imu_only.state.copy()
                
                if len(pure_predict_state) == 6:
                    pure_pred_x, pure_pred_y, pure_pred_heading, pure_pred_omega, pure_pred_velocity, pure_pred_accel = pure_predict_state
                elif len(pure_predict_state) == 4:
                    pure_pred_x, pure_pred_y, pure_pred_heading, pure_pred_velocity = pure_predict_state
                    pure_pred_omega = gyro
                    pure_pred_accel = accel
                else:
                    print(f"⚠️ Unexpected EKF state dimension: {len(pure_predict_state)}")
                    break
                
                pure_pred_heading_deg = math.degrees(pure_pred_heading)
                
                # ===== LOG PURE IMU PREDICT DATA DENGAN FILTER INFO =====
                imu_predict_data = [
                    current_time, dt_actual, 
                    accel, gyro, math.degrees(gyro),  # Filtered values
                    accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw values
                    pure_pred_x, pure_pred_y, pure_pred_heading_deg, pure_pred_velocity, pure_pred_accel,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    filter_stats.get('accel_cutoff', 0), filter_stats.get('gyro_cutoff', 0),
                    predict_counter
                ]
                imu_predict_queue.put(imu_predict_data)
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                last_time = current_time
                continue
            
            # ===== READ GPS =====
            gps_coords = gps_handler.get_coords()
            gps_valid = is_gps_valid(gps_coords)
            gps_updated = False
            
            # GPS initialization
            if gps_valid and not gps_initialized:
                lat_ref = gps_coords['latitude']
                lon_ref = gps_coords['longitude'] 
                alt_ref = gps_coords.get('altitude', 0)
                
                # Set initial position pada KEDUA EKF
                x_gps, y_gps = latlon_to_xy(lat_ref, lon_ref, alt_ref, 
                                           lat_ref, lon_ref, alt_ref)
                ekf_combined.state[0] = x_gps  # Should be 0
                ekf_combined.state[1] = y_gps  # Should be 0
                ekf_imu_only.state[0] = x_gps  # Should be 0  
                ekf_imu_only.state[1] = y_gps  # Should be 0
                
                gps_initialized = True
                print(f"GPS reference set: {lat_ref:.6f}, {lon_ref:.6f}")
            
            # ===== GPS UPDATE HANYA PADA COMBINED EKF =====
            if gps_valid and gps_initialized:
                try:
                    gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                               gps_coords['longitude'],
                                               gps_coords.get('altitude', alt_ref),
                                               lat_ref, lon_ref, alt_ref)
                    
                    gps_measurement = np.array([gps_x, gps_y])
                    ekf_combined.update(gps_measurement)  # ⚠️ HANYA combined EKF!
                    gps_update_counter += 1
                    gps_updated = True
                except Exception as e:
                    print(f"GPS update error: {e}")
            
            # Extract final combined state
            if len(ekf_combined.state) == 6:
                final_est_x, final_est_y, final_est_heading, final_est_omega, final_est_velocity, final_est_accel = ekf_combined.state
            elif len(ekf_combined.state) == 4:
                final_est_x, final_est_y, final_est_heading, final_est_velocity = ekf_combined.state
                final_est_omega = gyro
                final_est_accel = accel
            else:
                print(f"⚠️ Unexpected EKF state dimension: {len(ekf_combined.state)}")
                break
                
            final_est_heading_deg = math.degrees(final_est_heading)
            
            # ===== COMBINED LOGGING DENGAN FILTER INFO =====
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                log_data = [
                    current_time, dt_actual,
                    gps_x, gps_y,
                    gps_coords['latitude'], gps_coords['longitude'],
                    accel, gyro, math.degrees(gyro),  # Filtered values
                    accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw values untuk comparison
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    gps_updated,
                    predict_counter, gps_update_counter
                ]
            else:
                log_data = [
                    current_time, dt_actual,
                    None, None, None, None,
                    accel, gyro, math.degrees(gyro),  # Filtered values
                    accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw values
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    False,
                    predict_counter, gps_update_counter
                ]
            
            log_queue.put(log_data)
            
            # ===== LOG FILTER STATISTICS =====
            if filter_stats_log_counter % 20 == 0:  # Every second
                filter_stats_data = [
                    current_time,
                    filter_stats['accel_noise_reduction'],
                    filter_stats['gyro_noise_reduction'],
                    filter_stats.get('accel_cutoff', 0),
                    filter_stats.get('gyro_cutoff', 0),
                    abs(accel_raw - accel),  # Instantaneous noise removed
                    abs(gyro_raw - gyro),
                    iteration_count
                ]
                filter_stats_queue.put(filter_stats_data)
            
            filter_stats_log_counter += 1
            
            # ===== ENHANCED STATUS PRINT =====
            if iteration_count % 40 == 0:  # Every 2 seconds
                gps_stats = gps_handler.get_stats()
                imu_stats = imu_handler.get_stats()
                
                print(f"t={iteration_count*dt:.1f}s | "
                      f"GPS: {gps_stats['read_count']}/{gps_stats['error_count']} | "
                      f"IMU: {imu_stats['read_count']}/{imu_stats['error_count']} | "
                      f"Predicts: {predict_counter} | GPS updates: {gps_update_counter}")
                print(f"    IMU-only: ({pure_pred_x:.2f}, {pure_pred_y:.2f}) | "
                      f"Combined: ({final_est_x:.2f}, {final_est_y:.2f}) | "
                      f"Diff: ({abs(final_est_x-pure_pred_x):.2f}, {abs(final_est_y-pure_pred_y):.2f})")
                print(f"    Filter effectiveness: Accel={filter_stats['accel_noise_reduction']:.1%}, "
                      f"Gyro={filter_stats['gyro_noise_reduction']:.1%}")
                print(f"    Noise reduction: A={abs(accel_raw-accel):.3f}, G={abs(math.degrees(gyro_raw-gyro)):.1f}°/s")
                
                # Warning jika filter tidak efektif
                if filter_stats['accel_noise_reduction'] < 0.05:
                    print("    ⚠️ Low accel filter effectiveness - consider adjusting cutoff frequency")
                if filter_stats['gyro_noise_reduction'] < 0.05:
                    print("    ⚠️ Low gyro filter effectiveness - consider adjusting cutoff frequency")
                
                # Validasi: kedua hasil harus berbeda jika GPS aktif!
                if gps_initialized and abs(final_est_x - pure_pred_x) < 0.01 and abs(final_est_y - pure_pred_y) < 0.01:
                    print("    ⚠️ WARNING: IMU-only dan Combined state terlalu mirip! GPS mungkin tidak bekerja.")
            
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
        
        # Stop logging threads
        log_queue.put("STOP")
        imu_predict_queue.put("STOP")
        filter_stats_queue.put("STOP")
        
        # Wait for threads to finish
        for thread in [log_thread, imu_predict_thread, filter_stats_thread]:
            if thread.is_alive():
                thread.join(timeout=2)
        
        print(f"✅ Data logged to:")
        print(f"   Combined: {log_filename}")
        print(f"   IMU predict: {imu_predict_filename}")
        print(f"   Filter stats: {filter_stats_filename}")
        print("✅ Low-pass filtering successfully reduced engine vibration noise!")
        print("✅ Check filter_stats log untuk melihat efektivitas filtering")

def filter_stats_logging_func(q: Queue, filename: str):
    """Specialized logging untuk filter statistics"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'accel_cutoff_hz', 'gyro_cutoff_hz',
            'instantaneous_accel_noise_removed', 'instantaneous_gyro_noise_removed',
            'iteration'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def test_filter_effectiveness():
    """Test untuk melihat efektivitas filter dalam real-time"""
    print("=== Testing Filter Effectiveness in Real-Time ===")
    
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(
        application_type='vehicle',
        accel_filter_config={'cutoff': 2.0, 'type': 'butterworth', 'adaptive': False},
        gyro_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False}
    )
    time.sleep(2)
    
    print("Collecting data for 30 seconds...")
    print("Time\tAccel_Raw\tAccel_Filt\tNoise_Red\tGyro_Raw\tGyro_Filt\tNoise_Red")
    print("-" * 80)
    
    start_time = time.time()
    
    try:
        for i in range(300):  # 30 seconds at 10Hz
            accel_f, gyro_f, accel_r, gyro_r, stats = imu_handler.get_data(return_raw=True, return_stats=True)
            
            elapsed = time.time() - start_time
            
            if i % 10 == 0:  # Print every second
                accel_noise_red = abs(accel_r - accel_f)
                gyro_noise_red = abs(math.degrees(gyro_r - gyro_f))
                
                print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t{accel_noise_red:.3f}\t\t"
                      f"{math.degrees(gyro_r):.1f}°/s\t\t{math.degrees(gyro_f):.1f}°/s\t\t{gyro_noise_red:.1f}°/s")
            
            time.sleep(0.1)
        
        # Final effectiveness summary
        final_stats = imu_handler.get_stats()
        print(f"\nFilter Effectiveness Summary:")
        print(f"Accelerometer: {final_stats['filter_effectiveness']['accel_noise_reduction']:.1%} noise reduction")
        print(f"Gyroscope: {final_stats['filter_effectiveness']['gyro_noise_reduction']:.1%} noise reduction")
        print(f"IMU read success rate: {final_stats['success_rate']:.1%}")
        
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        imu_handler.stop()

def test_different_filter_settings():
    """Test berbagai konfigurasi filter untuk menemukan yang optimal"""
    print("=== Testing Different Filter Settings ===")
    
    test_configs = [
        {
            'name': 'Conservative (Low Cutoff)',
            'accel_cutoff': 1.0,
            'gyro_cutoff': 1.5,
            'filter_type': 'butterworth'
        },
        {
            'name': 'Vehicle Standard',
            'accel_cutoff': 3.0,
            'gyro_cutoff': 5.0,
            'filter_type': 'butterworth'
        },
        {
            'name': 'Aggressive (High Cutoff)',
            'accel_cutoff': 8.0,
            'gyro_cutoff': 12.0,
            'filter_type': 'butterworth'
        },
        {
            'name': 'Moving Average',
            'accel_cutoff': 3.0,
            'gyro_cutoff': 5.0,
            'filter_type': 'moving_average'
        },
        {
            'name': 'Exponential',
            'accel_cutoff': 3.0,
            'gyro_cutoff': 5.0,
            'filter_type': 'exponential'
        }
    ]
    
    results = []
    
    for config in test_configs:
        print(f"\nTesting: {config['name']}")
        
        try:
            imu_handler = FilteredIMUHandler(
                accel_filter_config={
                    'cutoff': config['accel_cutoff'],
                    'type': config['filter_type'],
                    'adaptive': False
                },
                gyro_filter_config={
                    'cutoff': config['gyro_cutoff'],
                    'type': config['filter_type'],
                    'adaptive': False
                }
            )
            
            time.sleep(3)  # Stabilization
            
            # Collect samples
            samples = []
            for i in range(100):  # 10 seconds at 10Hz
                accel_f, gyro_f, accel_r, gyro_r = imu_handler.get_data(return_raw=True)
                samples.append({
                    'accel_filtered': accel_f,
                    'gyro_filtered': gyro_f,
                    'accel_raw': accel_r,
                    'gyro_raw': gyro_r
                })
                time.sleep(0.1)
            
            # Calculate metrics
            if len(samples) > 10:
                accel_raw_std = np.std([s['accel_raw'] for s in samples])
                accel_filtered_std = np.std([s['accel_filtered'] for s in samples])
                gyro_raw_std = np.std([s['gyro_raw'] for s in samples])
                gyro_filtered_std = np.std([s['gyro_filtered'] for s in samples])
                
                accel_reduction = accel_raw_std / max(accel_filtered_std, 1e-6)
                gyro_reduction = gyro_raw_std / max(gyro_filtered_std, 1e-6)
                
                # Calculate delay (approximate)
                from lowpass_filters import calculate_filter_delay
                accel_delay = calculate_filter_delay(config['accel_cutoff'], 50.0, config['filter_type'])
                gyro_delay = calculate_filter_delay(config['gyro_cutoff'], 50.0, config['filter_type'])
                
                result = {
                    'name': config['name'],
                    'accel_reduction': accel_reduction,
                    'gyro_reduction': gyro_reduction,
                    'accel_delay_ms': accel_delay * 20,  # 20ms per sample at 50Hz
                    'gyro_delay_ms': gyro_delay * 20,
                    'config': config
                }
                results.append(result)
                
                print(f"  Accel noise reduction: {accel_reduction:.2f}x")
                print(f"  Gyro noise reduction: {gyro_reduction:.2f}x")
                print(f"  Estimated delay: {accel_delay * 20:.0f}ms (accel), {gyro_delay * 20:.0f}ms (gyro)")
            
            imu_handler.stop()
            time.sleep(1)
            
        except Exception as e:
            print(f"  Error: {e}")
    
    # Summary
    print(f"\n{'='*60}")
    print("FILTER COMPARISON SUMMARY")
    print(f"{'='*60}")
    print(f"{'Configuration':<20} {'Accel Red':<10} {'Gyro Red':<10} {'Delay (ms)':<12}")
    print("-" * 60)
    
    for result in results:
        print(f"{result['name']:<20} {result['accel_reduction']:<10.1f}x {result['gyro_reduction']:<10.1f}x "
              f"{result['accel_delay_ms']:<12.0f}")
    
    # Recommendation
    if results:
        best_overall = max(results, key=lambda x: (x['accel_reduction'] + x['gyro_reduction']) / 2)
        print(f"\nRECOMMENDED: {best_overall['name']}")
        print(f"  Best overall noise reduction with reasonable delay")

def test_imu_predict_only_filtered():
    """Test khusus IMU predict dengan filtering"""
    print("=== Testing Filtered IMU Predict Only ===")
    
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(application_type='vehicle')
    time.sleep(1)
    
    print("Initializing EKF...")
    dt = 0.05
    ekf = EKFSensorFusion(dt=dt)
    ekf.state[0] = 0  # x
    ekf.state[1] = 0  # y
    
    print("Running filtered IMU-only prediction...")
    print("Press Ctrl+C to stop")
    
    try:
        last_time = time.time()
        predict_counter = 0
        iteration_counter = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # Read filtered IMU data
            try:
                accel_f, gyro_f, accel_r, gyro_r, stats = imu_handler.get_data(return_raw=True, return_stats=True)
            except Exception as e:
                print(f"IMU read error: {e}")
                accel_f, gyro_f = 0, 0
                accel_r, gyro_r = 0, 0
                stats = {'accel_noise_reduction': 0, 'gyro_noise_reduction': 0}
            
            # EKF Predict dengan filtered data
            try:
                ekf.predict(imu_accel=accel_f, imu_omega=gyro_f, dt=dt_actual)
                predict_counter += 1
                
                # Extract state
                if len(ekf.state) >= 4:
                    pred_x, pred_y, pred_heading, pred_velocity = ekf.state[:4]
                else:
                    pred_x, pred_y = ekf.state[0], ekf.state[1]
                    pred_heading = 0
                    pred_velocity = 0
                
                pred_heading_deg = math.degrees(pred_heading)
                
                # Print status dengan filter info
                if iteration_counter % 20 == 0:
                    noise_accel = abs(accel_r - accel_f)
                    noise_gyro = abs(math.degrees(gyro_r - gyro_f))
                    
                    print(f"#{iteration_counter}: Pos({pred_x:.2f}, {pred_y:.2f}) | "
                          f"H:{pred_heading_deg:.1f}° | V:{pred_velocity:.2f}")
                    print(f"    Filter: A_noise={noise_accel:.3f} G_noise={noise_gyro:.1f}°/s | "
                          f"Effectiveness: {stats['accel_noise_reduction']:.1%}, {stats['gyro_noise_reduction']:.1%}")
                
            except Exception as e:
                print(f"EKF predict error: {e}")
            
            iteration_counter += 1
            last_time = current_time
            
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print(f"\nStopped after {iteration_counter} iterations")
    finally:
        imu_handler.stop()

def test_gps_only():
    """Test GPS handler only"""
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

def quick_filter_demo():
    """Quick demonstration of filter effectiveness"""
    print("=== Quick Filter Demo ===")
    print("Showing raw vs filtered IMU data for 10 seconds...")
    
    imu_handler = FilteredIMUHandler(
        application_type='vehicle',
        accel_filter_config={'cutoff': 2.0, 'type': 'butterworth'},
        gyro_filter_config={'cutoff': 3.0, 'type': 'butterworth'}
    )
    
    try:
        time.sleep(2)  # Stabilization
        
        print("Time\tAccel_Raw\tAccel_Filt\tGyro_Raw\tGyro_Filt")
        print("-" * 60)
        
        start_time = time.time()
        for i in range(50):  # 10 seconds at 5Hz
            accel_f, gyro_f, accel_r, gyro_r = imu_handler.get_data(return_raw=True)
            
            elapsed = time.time() - start_time
            print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t"
                  f"{math.degrees(gyro_r):.1f}°/s\t\t{math.degrees(gyro_f):.1f}°/s")
            
            time.sleep(0.2)
        
        print("\n✅ Demo completed! Filter reduces high-frequency noise.")
        
    except KeyboardInterrupt:
        print("\nDemo stopped")
    finally:
        imu_handler.stop()

# ===== RTK-SPECIFIC ANALYSIS FUNCTIONS =====

def analyze_rtk_accuracy():
    """Analyze RTK accuracy from logged data"""
    print("=== RTK Accuracy Analysis ===")
    print("This function would analyze the logged RTK data")
    print("Features to implement:")
    print("- RTK fix percentage")
    print("- Position accuracy over time")
    print("- RTCM correction effectiveness")
    print("- Compare standard GPS vs RTK precision")

def generate_accuracy_report():
    """Generate comprehensive accuracy report comparing all methods"""
    print("=== Generating Accuracy Report ===")
    print("This function would process all logged data and generate:")
    print("- EKF vs RTK accuracy statistics")
    print("- IMU-only drift analysis") 
    print("- Filter effectiveness metrics")
    print("- Recommendations for optimal settings")

# ===== ADDITIONAL UTILITY FUNCTIONS =====

def rtk_status_monitor():
    """Real-time RTK status monitoring"""
    print("=== RTK Status Monitor ===")
    
    rtk_handler = RTKGroundTruthHandler()
    rtk_handler.start()
    
    try:
        while True:
            rtk_coords = rtk_handler.get_rtk_coords()
            rtk_stats = rtk_handler.get_stats()
            
            print(f"\n📡 RTK Status - {datetime.now().strftime('%H:%M:%S')}")
            
            if rtk_coords:
                quality_names = {
                    0: "❌ Invalid",
                    1: "📡 GPS Standard", 
                    2: "🔸 DGPS",
                    3: "🔶 PPS",
                    4: "🎯 RTK Fixed",
                    5: "🔹 RTK Float",
                    6: "⚠️ Estimated"
                }
                
                quality = rtk_coords.get('quality', 0)
                quality_str = quality_names.get(quality, f"Unknown({quality})")
                
                print(f"Position: {rtk_coords['latitude']:.8f}°, {rtk_coords['longitude']:.8f}°")
                print(f"Quality: {quality_str}")
                print(f"Satellites: {rtk_coords.get('satellites', 0)}")
                print(f"HDOP: {rtk_coords.get('hdop', 99):.2f}")
                print(f"RTCM Age: {rtk_coords.get('rtcm_age', 0):.1f}s")
                
                if rtk_coords.get('diff_station'):
                    print(f"Base Station: {rtk_coords['diff_station']}")
                
            else:
                print("❌ No RTK data available")
            
            print(f"Stats: {rtk_stats['read_count']} reads, {rtk_stats['error_count']} errors")
            print(f"RTK Fixes: {rtk_stats.get('rtk_fix_count', 0)}")
            print(f"RTCM Packets: {rtk_stats.get('rtcm_count', 0)}")
            
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nRTK monitor stopped")
    finally:
        rtk_handler.stop()

if __name__ == "__main__":
    main()
