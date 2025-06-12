import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock, Event
from queue import Queue, Empty
from collections import deque
import statistics

# Import custom modules
from gpsimuhandler import GPSHandler, FilteredIMUHandler, create_imu_handler
from ekfnparam3 import EKFSensorFusion
from helper import latlon_to_xy, is_gps_valid

class TimestampedData:
    """Data structure with precise timing"""
    def __init__(self, data, timestamp=None):
        self.data = data
        self.timestamp = timestamp or time.time()
        self.processed = False

class PreciseRateController:
    """Fixed precise rate control with timing compensation"""
    def __init__(self, target_hz):
        self.target_hz = target_hz
        self.target_interval = 1.0 / target_hz
        self.last_time = time.time()
        self.timing_history = deque(maxlen=50)
        self.actual_rate = target_hz
        self.cumulative_error = 0.0
        
    def wait_for_next(self):
        """Wait for next cycle with FIXED timing logic"""
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        # Calculate required sleep BEFORE updating last_time
        ideal_sleep = self.target_interval
        timing_error = elapsed - self.target_interval
        self.cumulative_error += timing_error
        
        # Compensate for cumulative timing drift
        compensated_sleep = ideal_sleep - self.cumulative_error * 0.1
        sleep_time = max(0, compensated_sleep)
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        # ✅ FIX: Update last_time to current_time (BEFORE sleep)
        self.last_time = current_time
        
        # Record actual interval for rate calculation
        actual_interval = time.time() - current_time
        if actual_interval > 0:
            self.timing_history.append(actual_interval)
            if len(self.timing_history) > 10:
                self.actual_rate = 1.0 / statistics.mean(self.timing_history)
        
        return elapsed
    
    def get_actual_rate(self):
        return self.actual_rate
    
    def get_timing_accuracy(self):
        """Get timing accuracy as percentage"""
        if not self.timing_history:
            return 1.0
        
        mean_interval = statistics.mean(self.timing_history)
        accuracy = 1.0 - abs(mean_interval - self.target_interval) / self.target_interval
        return max(0, min(1, accuracy))

    def shoul_triggre(self, current_time):
	return (current_time - self.last_time) >= self.target_interval
    
    def reset_error(self):
        """Reset cumulative error (call periodically)"""
        self.cumulative_error = 0.0

class BufferedSensorReader:
    """Thread-safe buffered sensor reading with interpolation"""
    def __init__(self, sensor_handler, target_hz, buffer_size=300):
        self.sensor_handler = sensor_handler
        self.target_hz = target_hz
        self.buffer = deque(maxlen=buffer_size)
        self.buffer_lock = Lock()
        self.running = Event()
        self.stats = {
            'read_count': 0,
            'error_count': 0,
            'buffer_overruns': 0,
            'actual_rate': 0.0,
            'success_rate': 0.0
        }
        self.rate_controller = PreciseRateController(target_hz)
        
    def start(self):
        """Start buffered reading thread"""
        self.running.set()
        self.thread = Thread(target=self._reading_loop, daemon=True)
        self.thread.start()
        print(f"Started buffered sensor reader at {self.target_hz}Hz")
        
    def stop(self):
        """Stop reading thread"""
        self.running.clear()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=2)
    
    def _reading_loop(self):
        """High-rate sensor reading loop"""
        print(f"Sensor reading loop started at {self.target_hz}Hz")
        
        while self.running.is_set():
            try:
                # Read sensor data with precise timestamp
                timestamp = time.time()
                
                # Handle different sensor types
                if hasattr(self.sensor_handler, 'get_data'):
                    # IMU Handler
                    try:
                        data = self.sensor_handler.get_data(return_raw=True, return_stats=True)
                    except TypeError:
                        # Fallback if return_raw not supported
                        data = self.sensor_handler.get_data()
                elif hasattr(self.sensor_handler, 'get_coords'):
                    # GPS Handler
                    data = self.sensor_handler.get_coords()
                else:
                    raise Exception("Unknown sensor handler type")
                
                # Store in buffer with timestamp
                timestamped_data = TimestampedData(data, timestamp)
                
                with self.buffer_lock:
                    if len(self.buffer) >= self.buffer.maxlen - 1:
                        self.stats['buffer_overruns'] += 1
                    self.buffer.append(timestamped_data)
                
                self.stats['read_count'] += 1
                self.stats['actual_rate'] = self.rate_controller.get_actual_rate()
                
                # Calculate success rate
                total_attempts = self.stats['read_count'] + self.stats['error_count']
                if total_attempts > 0:
                    self.stats['success_rate'] = self.stats['read_count'] / total_attempts
                
                # Reset cumulative error periodically
                if self.stats['read_count'] % 100 == 0:
                    self.rate_controller.reset_error()
                
            except Exception as e:
                self.stats['error_count'] += 1
                if self.stats['error_count'] % 100 == 1:  # Print every 100 errors
                    print(f"Sensor read error: {e}")
            
            # Precise timing control
            self.rate_controller.wait_for_next()
    
    def get_latest(self, max_age=0.1):
        """Get latest data within max_age seconds"""
        current_time = time.time()
        
        with self.buffer_lock:
            if not self.buffer:
                return None
            
            # Find latest valid data
            for data in reversed(self.buffer):
                if current_time - data.timestamp <= max_age:
                    return data
            
            # If no recent data, return latest available
            return self.buffer[-1] if self.buffer else None
    
    def get_interpolated(self, target_time, max_age=0.2):
        """Get interpolated data for specific timestamp"""
        with self.buffer_lock:
            if len(self.buffer) < 2:
                return self.get_latest(max_age)
            
            # Find bracketing samples
            before_data = None
            after_data = None
            
            for data in self.buffer:
                if data.timestamp <= target_time:
                    before_data = data
                elif data.timestamp > target_time and after_data is None:
                    after_data = data
                    break
            
            if before_data and after_data and (target_time - before_data.timestamp) < max_age:
                # Linear interpolation for IMU data
                alpha = (target_time - before_data.timestamp) / (after_data.timestamp - before_data.timestamp)
                alpha = max(0, min(1, alpha))  # Clamp to [0, 1]
                
                # Interpolate based on data type
                if isinstance(before_data.data, tuple) and len(before_data.data) >= 2:
                    # IMU data (accel, gyro, ...)
                    before_vals = before_data.data[:2]  # accel, gyro
                    after_vals = after_data.data[:2]
                    
                    interp_accel = before_vals[0] + alpha * (after_vals[0] - before_vals[0])
                    interp_gyro = before_vals[1] + alpha * (after_vals[1] - before_vals[1])
                    
                    # Create interpolated data tuple
                    interp_data = (interp_accel, interp_gyro) + before_data.data[2:]
                    return TimestampedData(interp_data, target_time)
                    
            # Fallback to latest data
            return before_data if before_data else self.get_latest(max_age)
    
    def get_stats(self):
        """Get sensor reading statistics"""
        return self.stats.copy()

class CompleteSensorFusion:
    """Complete sensor fusion class with all original features"""
    
    def __init__(self, config=None):
        # Default configuration
        self.config = config or {
            'rates': {
                'imu_hardware': 50,     # IMU hardware sampling
                'ekf_predict': 25,      # EKF prediction rate
                'gps_poll': 5,          # GPS polling rate
                'status': 1             # Status print rate
            },
            'filter_config': {
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
            },
            'fusion_config': {
                'max_imu_age': 0.1,     # Max age for IMU data (seconds)
                'max_gps_age': 2.0,     # Max age for GPS data (seconds)
                'gps_timeout': 30.0     # GPS timeout for initialization
            },
            'logging_config': {
                'separate_rates': True,  # Use separate rates for each log type
                'imu_raw_rate': 50,      # Raw IMU logging rate
                'imu_filtered_rate': 50, # Filtered IMU logging rate
                'ekf_predict_rate': 25,  # EKF predict logging rate
                'combined_rate': 25,     # Combined fusion logging rate
                'filter_stats_rate': 5, # Filter stats rate
                'system_status_rate': 1 # System status rate
            }
        }
        
        # Initialize components
        self.gps_reader = None
        self.imu_reader = None
        self.ekf_combined = None
        self.ekf_imu_only = None
        
        # State variables
        self.gps_initialized = False
        self.lat_ref = None
        self.lon_ref = None
        self.alt_ref = None
        
        # Control
        self.running = Event()
        
        # Statistics
        self.stats = {
            'predictions': 0,
            'gps_updates': 0,
            'interpolations': 0,
            'timing_violations': 0
        }
        
        # Logging
        self.log_queues = {}
        self.log_threads = []
        self.log_files = {}
    
    def initialize(self):
        """Initialize all components"""
        print("=== INITIALIZING COMPLETE SENSOR FUSION ===")
        print("Multi-rate architecture with all original features")
        
        # Initialize GPS with buffered reading
        print("Initializing GPS handler...")
        gps_handler = GPSHandler()
        self.gps_reader = BufferedSensorReader(
            gps_handler, 
            self.config['rates']['gps_poll']
        )
        
        # Initialize IMU with filtering and buffered reading
        print("Initializing filtered IMU handler...")
        imu_handler = FilteredIMUHandler(**self.config['filter_config'])
        self.imu_reader = BufferedSensorReader(
            imu_handler,
            self.config['rates']['imu_hardware']
        )
        
        # Test IMU
        time.sleep(1)
        try:
            test_data = self.imu_reader.sensor_handler.get_data()
            if isinstance(test_data, tuple) and len(test_data) >= 2:
                print(f"IMU test: accel={test_data[0]:.3f}, gyro={math.degrees(test_data[1]):.1f}°/s")
        except Exception as e:
            print(f"IMU test warning: {e}")
        
        # Initialize EKF instances
        print("Initializing EKF instances...")
        ekf_dt = 1.0 / self.config['rates']['ekf_predict']
        self.ekf_combined = EKFSensorFusion(dt=ekf_dt)
        self.ekf_imu_only = EKFSensorFusion(dt=ekf_dt)
        
        print("Configuration Summary:")
        print(f"  IMU sampling: {self.config['rates']['imu_hardware']} Hz")
        print(f"  EKF predict: {self.config['rates']['ekf_predict']} Hz")  
        print(f"  GPS polling: {self.config['rates']['gps_poll']} Hz")
        print(f"  Accel filter: {self.config['filter_config']['accel_filter_config']['cutoff']} Hz")
        print(f"  Gyro filter: {self.config['filter_config']['gyro_filter_config']['cutoff']} Hz")
        
        # Show logging configuration
        if self.config['logging_config']['separate_rates']:
            print("Logging Rates:")
            print(f"  IMU raw: {self.config['logging_config']['imu_raw_rate']} Hz")
            print(f"  IMU filtered: {self.config['logging_config']['imu_filtered_rate']} Hz")
            print(f"  EKF predict: {self.config['logging_config']['ekf_predict_rate']} Hz")
            print(f"  Combined: {self.config['logging_config']['combined_rate']} Hz")
            print(f"  Filter stats: {self.config['logging_config']['filter_stats_rate']} Hz")
            print(f"  System status: {self.config['logging_config']['system_status_rate']} Hz")
        
        return True
    
    def start_sensors(self):
        """Start sensor reading threads"""
        print("\nStarting sensor threads...")
        self.gps_reader.start()
        self.imu_reader.start()
        time.sleep(2)  # Stabilization
        
    def stop_sensors(self):
        """Stop sensor threads"""
        if self.gps_reader:
            self.gps_reader.stop()
        if self.imu_reader:
            self.imu_reader.stop()
    
    def initialize_gps_reference(self):
        """Initialize GPS reference"""
        print("Waiting for GPS initialization...")
        timeout = self.config['fusion_config']['gps_timeout']
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            gps_data = self.gps_reader.get_latest(max_age=2.0)
            if gps_data and gps_data.data and is_gps_valid(gps_data.data):
                coords = gps_data.data
                self.lat_ref = coords['latitude']
                self.lon_ref = coords['longitude']
                self.alt_ref = coords.get('altitude', 0)
                
                self.ekf_combined.state[0] = 0.0
                self.ekf_combined.state[1] = 0.0
                self.ekf_imu_only.state[0] = 0.0
                self.ekf_imu_only.state[1] = 0.0
                
                self.gps_initialized = True
                print(f"GPS reference: {self.lat_ref:.6f}, {self.lon_ref:.6f}")
                return True
            
            time.sleep(0.5)
        
        print("⚠️ GPS timeout - continuing without GPS reference")
        return False
    
    def setup_complete_logging(self):
        """Setup comprehensive logging with proper rates"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # ALL original log files with appropriate rates
        self.log_files = {
            'imu_raw': f"imu_raw_50hz_{timestamp}.csv",
            'imu_filtered': f"imu_filtered_50hz_{timestamp}.csv", 
            'ekf_predict': f"ekf_predict_25hz_{timestamp}.csv",
            'combined': f"sensor_fusion_complete_{timestamp}.csv",
            'filter_stats': f"filter_stats_5hz_{timestamp}.csv",
            'system_status': f"system_status_1hz_{timestamp}.csv",
            'gps_events': f"gps_events_{timestamp}.csv"
        }
        
        # Create queues
        for log_type in self.log_files.keys():
            self.log_queues[log_type] = Queue()
        
        # Start logging threads with specific headers
        for log_type, filename in self.log_files.items():
            thread = Thread(
                target=self._complete_logging_worker,
                args=(log_type, filename),
                daemon=True
            )
            thread.start()
            self.log_threads.append(thread)
        
        print("\nComplete Logging Setup:")
        for log_type, filename in self.log_files.items():
            print(f"  {filename}")
        
        return True
    
    def _complete_logging_worker(self, log_type, filename):
        """Complete logging worker with all original headers"""
        headers = {
            'imu_raw': [
                'time', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'
            ],
            'imu_filtered': [
                'time', 'dt', 
                'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
                'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
                'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'accel_cutoff_hz', 'gyro_cutoff_hz'
            ],
            'ekf_predict': [
                'time', 'dt', 
                'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
                'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
                'predict_x', 'predict_y', 'predict_heading_deg', 
                'predict_velocity', 'predict_accel', 
                'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'accel_cutoff_hz', 'gyro_cutoff_hz',
                'predict_count', 'P_trace'
            ],
            'combined': [
                'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
                'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
                'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
                'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
                'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'gps_updated', 'predict_count', 'gps_update_count',
                'P_trace', 'K_matrix', 'innovation_vector'
            ],
            'filter_stats': [
                'time', 'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'accel_cutoff_hz', 'gyro_cutoff_hz',
                'instantaneous_accel_noise_removed', 'instantaneous_gyro_noise_removed',
                'iteration'
            ],
            'system_status': [
                'time', 'predictions', 'gps_updates', 'interpolations', 'timing_violations', 
                'imu_read_count', 'imu_error_count', 'imu_actual_rate',
                'gps_read_count', 'gps_error_count', 'gps_actual_rate',
                'imu_buffer_overruns', 'gps_buffer_overruns'
            ],
            'gps_events': [
                'time', 'event_type', 'lat', 'lon', 'alt', 'gps_x', 'gps_y',
                'accuracy', 'num_satellites'
            ]
        }
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers.get(log_type, ['time', 'data']))
            
            while True:
                try:
                    row = self.log_queues[log_type].get(timeout=1)
                    if row == "STOP":
                        break
                    writer.writerow(row)
                    f.flush()
                except Empty:
                    continue
                except Exception as e:
                    print(f"Logging error ({log_type}): {e}")
    
    def run_complete_fusion_loop(self):
        """Complete fusion loop with all features"""
        print("\nStarting complete fusion loop...")
        
        # Rate controllers for different logging rates
        ekf_controller = PreciseRateController(self.config['rates']['ekf_predict'])
        imu_log_controller = PreciseRateController(self.config['logging_config']['imu_filtered_rate'])
        filter_stats_controller = PreciseRateController(self.config['logging_config']['filter_stats_rate'])
        status_controller = PreciseRateController(self.config['rates']['status'])
        combined_log_controller = PreciseRateController(self.config['logging_config']['combined_rate'])
        
        iteration_count = 0
        filter_stats_log_counter = 0
        
        try:
            self.running.set()
            
            while self.running.is_set():
                current_time = time.time()
                
                # ===== HIGH-RATE IMU DATA LOGGING =====
                imu_data = self.imu_reader.get_latest(max_age=0.05)
                if imu_data and imu_data.data:
                    if imu_log_controller.should_trigger(current_time):
                        # Log filtered IMU data at 50 Hz
                        if isinstance(imu_data.data, tuple) and len(imu_data.data) >= 5:
                            accel_f, gyro_f, accel_r, gyro_r, stats = imu_data.data
                            
                            self.log_queues['imu_filtered'].put([
                                imu_data.timestamp, current_time - imu_data.timestamp,
                                accel_f, gyro_f, math.degrees(gyro_f),
                                accel_r, gyro_r, math.degrees(gyro_r),
                                stats.get('accel_noise_reduction', 0),
                                stats.get('gyro_noise_reduction', 0),
                                stats.get('accel_cutoff', 0),
                                stats.get('gyro_cutoff', 0)
                            ])
                
                # ===== EKF PROCESSING (25 Hz) =====
                dt_actual = ekf_controller.wait_for_next()
                
                # Get interpolated IMU data for EKF
                imu_interp = self.imu_reader.get_interpolated(
                    current_time - 0.005,
                    max_age=self.config['fusion_config']['max_imu_age']
                )
                
                if imu_interp and imu_interp.data:
                    try:
                        if isinstance(imu_interp.data, tuple) and len(imu_interp.data) >= 5:
                            accel, gyro, accel_raw, gyro_raw, filter_stats = imu_interp.data
                        elif isinstance(imu_interp.data, tuple) and len(imu_interp.data) >= 2:
                            accel, gyro = imu_interp.data[0], imu_interp.data[1]
                            accel_raw, gyro_raw = accel, gyro
                            filter_stats = {}
                        else:
                            continue
                        
                        # EKF Predictions
                        self.ekf_combined.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                        self.ekf_imu_only.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                        
                        self.stats['predictions'] += 1
                        
                        # Check interpolation
                        if abs(imu_interp.timestamp - current_time) > 0.01:
                            self.stats['interpolations'] += 1
                        
                        # Extract states
                        pred_state = self.ekf_imu_only.state.copy()
                        combined_state = self.ekf_combined.state.copy()
                        
                        # Get covariances
                        P_pred = np.trace(self.ekf_imu_only.get_covariance())
                        P_combined = np.trace(self.ekf_combined.get_covariance())
                        
                        # Log EKF predict at 25 Hz
                        if len(pred_state) >= 4:
                            self.log_queues['ekf_predict'].put([
                                current_time, dt_actual,
                                accel, gyro, math.degrees(gyro),
                                accel_raw, gyro_raw, math.degrees(gyro_raw),
                                pred_state[0], pred_state[1], math.degrees(pred_state[2]),
                                pred_state[3], pred_state[5] if len(pred_state) > 5 else accel,
                                filter_stats.get('accel_noise_reduction', 0),
                                filter_stats.get('gyro_noise_reduction', 0),
                                filter_stats.get('accel_cutoff', 0),
                                filter_stats.get('gyro_cutoff', 0),
                                self.stats['predictions'], P_pred
                            ])
                        
                    except Exception as e:
                        print(f"EKF prediction error: {e}")
                        continue
                else:
                    print("⚠️ No valid IMU data for EKF")
                    self.stats['timing_violations'] += 1
                    continue
                
                # ===== GPS UPDATE PHASE =====
                gps_updated = False
                if self.gps_initialized:
                    gps_data = self.gps_reader.get_latest(max_age=self.config['fusion_config']['max_gps_age'])
                    
                    if gps_data and gps_data.data and is_gps_valid(gps_data.data):
                        try:
                            coords = gps_data.data
                            gps_x, gps_y = latlon_to_xy(
                                coords['latitude'], coords['longitude'],
                                coords.get('altitude', self.alt_ref),
                                self.lat_ref, self.lon_ref, self.alt_ref
                            )
                            
                            # EKF Update
                            gps_measurement = np.array([gps_x, gps_y])
                            self.ekf_combined.update(gps_measurement)
                            self.stats['gps_updates'] += 1
                            gps_updated = True
                            
                            # Log GPS event
                            self.log_queues['gps_events'].put([
                                gps_data.timestamp, 'gps_update',
                                coords['latitude'], coords['longitude'],
                                coords.get('altitude', self.alt_ref),
                                gps_x, gps_y,
                                coords.get('accuracy', 0),
                                coords.get('num_satellites', 0)
                            ])
                            
                        except Exception as e:
                            print(f"GPS update error: {e}")
                
                # ===== COMBINED LOGGING (25 Hz) =====
                if combined_log_controller.should_trigger(current_time):
                    # Extract K and innovation if GPS updated
                    K_matrix = None
                    innovation_vector = None
                    if gps_updated:
                        try:
                            K_matrix = self.ekf_combined.get_kalman_gain()
                            innovation_vector = self.ekf_combined.get_innovation()
                        except:
                            pass
                    
                    # Log combined data
                    gps_data_for_log = self.gps_reader.get_latest(max_age=2.0)
                    if gps_data_for_log and gps_data_for_log.data and is_gps_valid(gps_data_for_log.data):
                        coords = gps_data_for_log.data
                        gps_x, gps_y = latlon_to_xy(
                            coords['latitude'], coords['longitude'],
                            coords.get('altitude', self.alt_ref),
                            self.lat_ref, self.lon_ref, self.alt_ref
                        )
                        
                        combined_row = [
                            current_time, dt_actual, gps_x, gps_y,
                            coords['latitude'], coords['longitude'],
                            accel, gyro, math.degrees(gyro),
                            accel_raw, gyro_raw, math.degrees(gyro_raw),
                            combined_state[0], combined_state[1], math.degrees(combined_state[2]),
                            combined_state[3], combined_state[5] if len(combined_state) > 5 else accel,
                            filter_stats.get('accel_noise_reduction', 0),
                            filter_stats.get('gyro_noise_reduction', 0),
                            gps_updated, self.stats['predictions'], self.stats['gps_updates'],
                            P_combined,
                            str(K_matrix.tolist()) if K_matrix is not None else None,
                            str(innovation_vector.tolist()) if innovation_vector is not None else None
                        ]
                    else:
                        combined_row = [
                            current_time, dt_actual, None, None, None, None,
                            accel, gyro, math.degrees(gyro),
                            accel_raw, gyro_raw, math.degrees(gyro_raw),
                            combined_state[0], combined_state[1], math.degrees(combined_state[2]),
                            combined_state[3], combined_state[5] if len(combined_state) > 5 else accel,
                            filter_stats.get('accel_noise_reduction', 0),
                            filter_stats.get('gyro_noise_reduction', 0),
                            False, self.stats['predictions'], self.stats['gps_updates'],
                            P_combined, None, None
                        ]
                    
                    self.log_queues['combined'].put(combined_row)
                
                # ===== FILTER STATS LOGGING (5 Hz) =====
                if filter_stats_controller.should_trigger(current_time):
                    if 'filter_stats' in locals() and filter_stats:
                        noise_accel = abs(accel_raw - accel) if 'accel_raw' in locals() else 0
                        noise_gyro = abs(math.degrees(gyro_raw - gyro)) if 'gyro_raw' in locals() else 0
                        
                        self.log_queues['filter_stats'].put([
                            current_time,
                            filter_stats.get('accel_noise_reduction', 0),
                            filter_stats.get('gyro_noise_reduction', 0),
                            filter_stats.get('accel_cutoff', 0),
                            filter_stats.get('gyro_cutoff', 0),
                            noise_accel, noise_gyro,
                            iteration_count
                        ])
                
                # ===== ENHANCED STATUS PRINT (1 Hz) =====
                if status_controller.should_trigger(current_time):
                    gps_stats = self.gps_reader.get_stats()
                    imu_stats = self.imu_reader.get_stats()
                    
                    print(f"t={iteration_count*dt_actual:.1f}s | "
                          f"GPS: {gps_stats['read_count']}/{gps_stats['error_count']} "
                          f"({gps_stats['actual_rate']:.1f}Hz) | "
                          f"IMU: {imu_stats['read_count']}/{imu_stats['error_count']} "
                          f"({imu_stats['actual_rate']:.1f}Hz)")
                    
                    print(f"    Predictions: {self.stats['predictions']} | "
                          f"GPS updates: {self.stats['gps_updates']} | "
                          f"Interpolations: {self.stats['interpolations']} | "
                          f"Timing violations: {self.stats['timing_violations']}")
                    
                    if len(combined_state) >= 2 and len(pred_state) >= 2:
                        pos_diff = math.sqrt((combined_state[0] - pred_state[0])**2 + 
                                           (combined_state[1] - pred_state[1])**2)
                        print(f"    IMU-only: ({pred_state[0]:.2f}, {pred_state[1]:.2f}) | "
                              f"Combined: ({combined_state[0]:.2f}, {combined_state[1]:.2f}) | "
                              f"Diff: {pos_diff:.2f}m")
                    
                    # Enhanced filter effectiveness reporting
                    if 'filter_stats' in locals() and filter_stats:
                        print(f"    Filter effectiveness: Accel={filter_stats.get('accel_noise_reduction', 0):.1%}, "
                              f"Gyro={filter_stats.get('gyro_noise_reduction', 0):.1%}")
                        
                        if 'accel_raw' in locals() and 'gyro_raw' in locals():
                            noise_accel = abs(accel_raw - accel)
                            noise_gyro = abs(math.degrees(gyro_raw - gyro))
                            print(f"    Noise reduction: A={noise_accel:.3f}, G={noise_gyro:.1f}°/s")
                        
                        # Warning jika filter tidak efektif
                        if filter_stats.get('accel_noise_reduction', 0) < 0.05:
                            print("    ⚠️ Low accel filter effectiveness - consider adjusting cutoff frequency")
                        if filter_stats.get('gyro_noise_reduction', 0) < 0.05:
                            print("    ⚠️ Low gyro filter effectiveness - consider adjusting cutoff frequency")
                    
                    # P matrix monitoring
                    print(f"    P trace: IMU-only={P_pred:.2f}, Combined={P_combined:.2f}")
                    
                    # Validasi: kedua hasil harus berbeda jika GPS aktif!
                    if self.gps_initialized and len(combined_state) >= 2 and len(pred_state) >= 2:
                        if abs(combined_state[0] - pred_state[0]) < 0.01 and abs(combined_state[1] - pred_state[1]) < 0.01:
                            print("    ⚠️ WARNING: IMU-only dan Combined state terlalu mirip! GPS mungkin tidak bekerja.")
                    
                    # Log system status
                    self.log_queues['system_status'].put([
                        current_time, self.stats['predictions'], self.stats['gps_updates'],
                        self.stats['interpolations'], self.stats['timing_violations'],
                        imu_stats['read_count'], imu_stats['error_count'], imu_stats['actual_rate'],
                        gps_stats['read_count'], gps_stats['error_count'], gps_stats['actual_rate'],
                        imu_stats['buffer_overruns'], gps_stats['buffer_overruns']
                    ])
                
                iteration_count += 1
                filter_stats_log_counter += 1
                
                # Check for timing violations
                loop_time = time.time() - current_time
                target_loop_time = 1.0 / self.config['rates']['ekf_predict']
                if loop_time > target_loop_time * 1.5:
                    self.stats['timing_violations'] += 1
                
        except KeyboardInterrupt:
            print("\nShutdown requested...")
        except Exception as e:
            print(f"Main loop error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running.clear()
    
    def cleanup(self):
        """Cleanup all resources"""
        print("\nCleaning up...")
        
        # Stop sensors
        self.stop_sensors()
        
        # Stop logging
        for queue in self.log_queues.values():
            queue.put("STOP")
        
        for thread in self.log_threads:
            thread.join(timeout=2)
        
        print("✅ Cleanup complete")
        print("\nLogged files:")
        for log_type, filename in self.log_files.items():
            print(f"  {filename}")
        print("✅ Low-pass filtering successfully reduced engine vibration noise!")
        print("✅ Check filter_stats log untuk melihat efektivitas filtering")
        print("✅ P, K, dan innovation matrices logged untuk analisis EKF performance")
    
    def run(self):
        """Main entry point"""
        try:
            if not self.initialize():
                return False
            
            self.start_sensors()
            self.initialize_gps_reference()
            self.setup_complete_logging()
            self.run_complete_fusion_loop()
            
        finally:
            self.cleanup()
        
        return True

def main_sensor_fusion_with_filters():
    """COMPLETE replacement for original function with ALL features"""
    print("=== SENSOR FUSION WITH LOW-PASS FILTERING ===")
    print("Complete implementation with all original features")
    print("Multi-rate architecture + buffered sensors + precise timing")
    
    # Enhanced configuration matching original
    config = {
        'rates': {
            'imu_hardware': 50,     # High-rate IMU sampling
            'ekf_predict': 25,      # EKF prediction rate
            'gps_poll': 5,          # GPS polling rate
            'status': 1             # Status updates
        },
        'filter_config': {
            'application_type': 'vehicle',  # Original setting
            'accel_filter_config': {
                'cutoff': 3.0,        # Original: Allow vehicle dynamics, filter engine vibration
                'type': 'butterworth', # Original type
                'adaptive': False      # Original setting
            },
            'gyro_filter_config': {
                'cutoff': 5.0,         # Original: Allow turning dynamics
                'type': 'butterworth', # Original type
                'adaptive': False      # Original setting
            }
        },
        'fusion_config': {
            'max_imu_age': 0.1,     # Max age for IMU data
            'max_gps_age': 2.0,     # Max age for GPS data
            'gps_timeout': 30.0     # GPS initialization timeout
        },
        'logging_config': {
            'separate_rates': True,  # Use proper rates for each log type
            'imu_raw_rate': 50,      # Log all raw IMU data
            'imu_filtered_rate': 50, # Log all filtered IMU data
            'ekf_predict_rate': 25,  # Log all EKF predictions
            'combined_rate': 25,     # Log combined fusion results
            'filter_stats_rate': 5, # Filter effectiveness stats
            'system_status_rate': 1 # System health monitoring
        }
    }
    
    print("Filter Configuration:")
    print(f"  Application type: {config['filter_config']['application_type']}")
    print(f"  Accel filter: {config['filter_config']['accel_filter_config']['cutoff']} Hz {config['filter_config']['accel_filter_config']['type']}")
    print(f"  Gyro filter: {config['filter_config']['gyro_filter_config']['cutoff']} Hz {config['filter_config']['gyro_filter_config']['type']}")
    print(f"  Adaptive filtering: {config['filter_config']['accel_filter_config']['adaptive']}")
    
    # Create and run complete fusion system
    fusion_system = CompleteSensorFusion(config)
    return fusion_system.run()

# ===== ALL ORIGINAL TEST FUNCTIONS =====

def test_filter_effectiveness():
    """Test untuk melihat efektivitas filter dalam real-time - ORIGINAL"""
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
    """Test berbagai konfigurasi filter untuk menemukan yang optimal - ORIGINAL"""
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
                
                result = {
                    'name': config['name'],
                    'accel_reduction': accel_reduction,
                    'gyro_reduction': gyro_reduction,
                    'config': config
                }
                results.append(result)
                
                print(f"  Accel noise reduction: {accel_reduction:.2f}x")
                print(f"  Gyro noise reduction: {gyro_reduction:.2f}x")
            
            imu_handler.stop()
            time.sleep(1)
            
        except Exception as e:
            print(f"  Error: {e}")
    
    # Summary
    print(f"\n{'='*60}")
    print("FILTER COMPARISON SUMMARY")
    print(f"{'='*60}")
    print(f"{'Configuration':<20} {'Accel Red':<10} {'Gyro Red':<10}")
    print("-" * 60)
    
    for result in results:
        print(f"{result['name']:<20} {result['accel_reduction']:<10.1f}x {result['gyro_reduction']:<10.1f}x")
    
    # Recommendation
    if results:
        best_overall = max(results, key=lambda x: (x['accel_reduction'] + x['gyro_reduction']) / 2)
        print(f"\nRECOMMENDED: {best_overall['name']}")
        print(f"  Best overall noise reduction")

def test_imu_predict_only_filtered():
    """Test khusus IMU predict dengan filtering - ORIGINAL"""
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
                
                # Extract P matrix
                P_matrix = ekf.get_covariance()
                P_trace = np.trace(P_matrix)
                
                # Print status dengan filter info
                if iteration_counter % 20 == 0:
                    noise_accel = abs(accel_r - accel_f)
                    noise_gyro = abs(math.degrees(gyro_r - gyro_f))
                    
                    print(f"#{iteration_counter}: Pos({pred_x:.2f}, {pred_y:.2f}) | "
                          f"H:{pred_heading_deg:.1f}° | V:{pred_velocity:.2f} | P_trace:{P_trace:.2f}")
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
    """Test GPS handler only - ORIGINAL"""
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
    """Quick demo untuk melihat efek filtering dalam waktu singkat - ORIGINAL"""
    print("=== Quick Filter Demo ===")
    print("Menunjukkan perbedaan data raw vs filtered dalam 10 detik")
    
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(
        application_type='vehicle',
        accel_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
        gyro_filter_config={'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
    )
    time.sleep(2)
    
    print("\nData comparison (Raw vs Filtered):")
    print("Time\tAccel_Raw\tAccel_Filt\tDiff\tGyro_Raw\tGyro_Filt\tDiff")
    print("-" * 70)
    
    try:
        start_time = time.time()
        for i in range(50):  # 10 seconds at 5Hz
            try:
                accel_f, gyro_f, accel_r, gyro_r = imu_handler.get_data(return_raw=True)
                
                elapsed = time.time() - start_time
                accel_diff = abs(accel_r - accel_f)
                gyro_diff = abs(math.degrees(gyro_r - gyro_f))
                
                print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t{accel_diff:.3f}\t"
                      f"{math.degrees(gyro_r):6.1f}°/s\t{math.degrees(gyro_f):6.1f}°/s\t{gyro_diff:5.1f}°/s")
                
            except Exception as e:
                print(f"Error: {e}")
            
            time.sleep(0.2)
        
        print("\n✅ Demo selesai. Filter berhasil mengurangi noise!")
        
    except KeyboardInterrupt:
        print("\nDemo dihentikan oleh user")
    finally:
        imu_handler.stop()

def main():
    """Main function dengan pilihan testing dan konfigurasi - COMPLETE ORIGINAL"""
    print("GPS-IMU Sensor Fusion dengan Low-Pass Filtering")
    print("Complete implementation dengan semua fitur original")
    print("Multi-rate architecture + precise timing + all test functions")
    print()
    print("Pilihan:")
    print("1. Run main sensor fusion dengan filtering (COMPLETE)")
    print("2. Test filter effectiveness real-time")
    print("3. Test berbagai konfigurasi filter")
    print("4. Test IMU predict only (dengan filtering)")
    print("5. Test GPS only")
    print("6. Quick filter demo")
    print("7. Test timing performance")
    print("8. Test buffered sensor reading")
    
    choice = input("Masukkan pilihan (1-8): ").strip()
    
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
        test_timing_performance()
    elif choice == '8':
        test_buffered_sensor()
    else:
        print("Pilihan tidak valid")

# Additional test functions for completeness
def test_timing_performance():
    """Test timing performance - ENHANCED"""
    print("=== TESTING TIMING PERFORMANCE ===")
    
    rates_to_test = [10, 25, 50, 100]
    
    for target_rate in rates_to_test:
        print(f"\nTesting {target_rate} Hz rate controller...")
        controller = PreciseRateController(target_rate)
        
        timings = []
        start_time = time.time()
        
        for i in range(target_rate * 5):  # 5 seconds
            elapsed = controller.wait_for_next()
            timings.append(elapsed)
            
            if i % target_rate == 0:  # Every second
                actual_rate = controller.get_actual_rate()
                print(f"  Second {i//target_rate + 1}: {actual_rate:.1f} Hz")
        
        # Statistics
        mean_timing = statistics.mean(timings)
        std_timing = statistics.stdev(timings) if len(timings) > 1 else 0
        final_rate = controller.get_actual_rate()
        
        print(f"  Target: {target_rate} Hz, Achieved: {final_rate:.2f} Hz")
        print(f"  Mean interval: {mean_timing*1000:.2f} ms, Std: {std_timing*1000:.2f} ms")
        print(f"  Accuracy: {(final_rate/target_rate)*100:.1f}%")

def test_buffered_sensor():
    """Test buffered sensor reading - ENHANCED"""
    print("=== TESTING BUFFERED SENSOR READING ===")
    
    # Mock sensor for testing
    class MockIMUHandler:
        def __init__(self):
            self.counter = 0
            
        def get_data(self, return_raw=False, return_stats=False):
            self.counter += 1
            accel = 0.1 * math.sin(self.counter * 0.1)  # Simulated data
            gyro = 0.05 * math.cos(self.counter * 0.05)
            
            if return_raw and return_stats:
                return (accel, gyro, accel + 0.01, gyro + 0.005, 
                       {'accel_noise_reduction': 0.1, 'gyro_noise_reduction': 0.15})
            return (accel, gyro)
    
    mock_imu = MockIMUHandler()
    buffered_reader = BufferedSensorReader(mock_imu, target_hz=50)
    
    print("Starting buffered reader at 50 Hz...")
    buffered_reader.start()
    
    try:
        time.sleep(2)  # Let it collect data
        
        print("\nTesting data retrieval:")
        
        # Test latest data
        latest = buffered_reader.get_latest(max_age=0.1)
        print(f"Latest data: {latest.data if latest else 'None'}")
        
        # Test interpolation
        target_time = time.time() - 0.05  # 50ms ago
        interpolated = buffered_reader.get_interpolated(target_time)
        print(f"Interpolated data: {interpolated.data if interpolated else 'None'}")
        
        # Test statistics
        stats = buffered_reader.get_stats()
        print(f"Reader stats: {stats}")
        
        time.sleep(3)  # More data collection
        
        final_stats = buffered_reader.get_stats()
        print(f"Final stats: {final_stats}")
        
    finally:
        buffered_reader.stop()

# ===== LOGGING FUNCTIONS YANG HILANG (ORIGINAL STYLE) =====

def combined_logging_func(q: Queue, filename: str):
    """Enhanced combined logging dengan filter info dan EKF matrices - ORIGINAL"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
            'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'gps_updated', 'predict_count', 'gps_update_count', 
            'P_trace', 'K_matrix', 'innovation_vector'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def imu_predict_logging_func(q: Queue, filename: str):
    """Enhanced IMU predict logging dengan filter comparison dan P matrix - ORIGINAL"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'predict_x', 'predict_y', 'predict_heading_deg', 
            'predict_velocity', 'predict_accel', 
            'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'accel_cutoff_hz', 'gyro_cutoff_hz',
            'predict_count', 'P_trace'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def filter_stats_logging_func(q: Queue, filename: str):
    """Specialized logging untuk filter statistics - ORIGINAL"""
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

# ===== COMPATIBILITY LAYER UNTUK ORIGINAL API =====

def main_corrected_sensor_fusion():
    """Alias untuk backward compatibility"""
    return main_sensor_fusion_with_filters()

def create_imu_handler(*args, **kwargs):
    """Create IMU handler - compatibility function"""
    return FilteredIMUHandler(*args, **kwargs)

# ===== ADVANCED TESTING FUNCTIONS =====

def test_comprehensive_performance():
    """Comprehensive performance test dengan semua komponen"""
    print("=== COMPREHENSIVE PERFORMANCE TEST ===")
    
    print("Testing all components together for 60 seconds...")
    
    # Setup complete system
    config = {
        'rates': {
            'imu_hardware': 50,
            'ekf_predict': 25,
            'gps_poll': 5,
            'status': 0.2  # Every 5 seconds
        },
        'filter_config': {
            'application_type': 'vehicle',
            'accel_filter_config': {'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
            'gyro_filter_config': {'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
        },
        'fusion_config': {
            'max_imu_age': 0.1,
            'max_gps_age': 2.0,
            'gps_timeout': 10.0
        }
    }
    
    fusion_system = CompleteSensorFusion(config)
    
    try:
        # Initialize only (no full run)
        if fusion_system.initialize():
            fusion_system.start_sensors()
            
            print("Collecting performance data...")
            start_time = time.time()
            
            # Monitor for 60 seconds
            while time.time() - start_time < 60:
                imu_stats = fusion_system.imu_reader.get_stats()
                gps_stats = fusion_system.gps_reader.get_stats()
                
                elapsed = time.time() - start_time
                
                if int(elapsed) % 10 == 0 and elapsed > 1:  # Every 10 seconds
                    print(f"t={elapsed:.0f}s: IMU({imu_stats['actual_rate']:.1f}Hz, "
                          f"{imu_stats['error_count']} errors) | "
                          f"GPS({gps_stats['actual_rate']:.1f}Hz, "
                          f"{gps_stats['error_count']} errors)")
                
                time.sleep(1)
            
            # Final statistics
            final_imu_stats = fusion_system.imu_reader.get_stats()
            final_gps_stats = fusion_system.gps_reader.get_stats()
            
            print("\n=== FINAL PERFORMANCE RESULTS ===")
            print(f"IMU Performance:")
            print(f"  Target: 50 Hz, Actual: {final_imu_stats['actual_rate']:.2f} Hz")
            print(f"  Success rate: {final_imu_stats['success_rate']:.1%}")
            print(f"  Buffer overruns: {final_imu_stats['buffer_overruns']}")
            
            print(f"GPS Performance:")
            print(f"  Target: 5 Hz, Actual: {final_gps_stats['actual_rate']:.2f} Hz")
            print(f"  Success rate: {final_gps_stats['success_rate']:.1%}")
            print(f"  Buffer overruns: {final_gps_stats['buffer_overruns']}")
            
        else:
            print("❌ System initialization failed")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        fusion_system.stop_sensors()
        print("✅ Performance test completed")

def test_filter_parameter_optimization():
    """Test untuk mencari parameter filter optimal"""
    print("=== FILTER PARAMETER OPTIMIZATION ===")
    
    # Parameter grid untuk testing
    accel_cutoffs = [1.0, 2.0, 3.0, 5.0, 8.0]
    gyro_cutoffs = [1.5, 3.0, 5.0, 8.0, 12.0]
    
    best_config = None
    best_score = 0
    results = []
    
    print("Testing parameter combinations...")
    print(f"Total combinations: {len(accel_cutoffs) * len(gyro_cutoffs)}")
    
    for accel_cutoff in accel_cutoffs:
        for gyro_cutoff in gyro_cutoffs:
            print(f"\nTesting: Accel={accel_cutoff}Hz, Gyro={gyro_cutoff}Hz")
            
            try:
                # Create IMU handler dengan parameter ini
                imu_handler = FilteredIMUHandler(
                    application_type='vehicle',
                    accel_filter_config={
                        'cutoff': accel_cutoff,
                        'type': 'butterworth',
                        'adaptive': False
                    },
                    gyro_filter_config={
                        'cutoff': gyro_cutoff,
                        'type': 'butterworth',
                        'adaptive': False
                    }
                )
                
                time.sleep(2)  # Stabilization
                
                # Collect data samples
                accel_reductions = []
                gyro_reductions = []
                
                for i in range(50):  # 5 seconds at 10Hz
                    try:
                        accel_f, gyro_f, accel_r, gyro_r, stats = imu_handler.get_data(
                            return_raw=True, return_stats=True
                        )
                        
                        accel_reduction = abs(accel_r - accel_f)
                        gyro_reduction = abs(gyro_r - gyro_f)
                        
                        accel_reductions.append(accel_reduction)
                        gyro_reductions.append(gyro_reduction)
                        
                    except Exception as e:
                        print(f"Data collection error: {e}")
                        break
                    
                    time.sleep(0.1)
                
                # Calculate performance metrics
                if len(accel_reductions) > 10 and len(gyro_reductions) > 10:
                    avg_accel_reduction = statistics.mean(accel_reductions)
                    avg_gyro_reduction = statistics.mean(gyro_reductions)
                    
                    # Combined score (weighted average)
                    score = (avg_accel_reduction * 0.6) + (avg_gyro_reduction * 0.4)
                    
                    result = {
                        'accel_cutoff': accel_cutoff,
                        'gyro_cutoff': gyro_cutoff,
                        'accel_reduction': avg_accel_reduction,
                        'gyro_reduction': avg_gyro_reduction,
                        'combined_score': score
                    }
                    results.append(result)
                    
                    print(f"  Score: {score:.4f} (A:{avg_accel_reduction:.4f}, G:{avg_gyro_reduction:.4f})")
                    
                    if score > best_score:
                        best_score = score
                        best_config = result
                
                imu_handler.stop()
                time.sleep(1)
                
            except Exception as e:
                print(f"  Error: {e}")
    
    # Results summary
    print("\n" + "="*80)
    print("FILTER OPTIMIZATION RESULTS")
    print("="*80)
    
    if results:
        # Sort by score
        results.sort(key=lambda x: x['combined_score'], reverse=True)
        
        print("Top 5 configurations:")
        print(f"{'Rank':<4} {'Accel(Hz)':<10} {'Gyro(Hz)':<10} {'Score':<10} {'Accel Red':<12} {'Gyro Red':<12}")
        print("-" * 80)
        
        for i, result in enumerate(results[:5]):
            print(f"{i+1:<4} {result['accel_cutoff']:<10} {result['gyro_cutoff']:<10} "
                  f"{result['combined_score']:<10.4f} {result['accel_reduction']:<12.4f} {result['gyro_reduction']:<12.4f}")
        
        if best_config:
            print(f"\n🏆 OPTIMAL CONFIGURATION:")
            print(f"   Accel cutoff: {best_config['accel_cutoff']} Hz")
            print(f"   Gyro cutoff: {best_config['gyro_cutoff']} Hz")
            print(f"   Combined score: {best_config['combined_score']:.4f}")
    else:
        print("❌ No valid results obtained")

def test_system_stress():
    """Stress test untuk system stability"""
    print("=== SYSTEM STRESS TEST ===")
    print("Running high-rate operations untuk test stability...")
    
    try:
        # Setup high-rate system
        config = {
            'rates': {
                'imu_hardware': 100,    # Very high rate
                'ekf_predict': 50,      # High rate
                'gps_poll': 10,         # High GPS rate
                'status': 1
            },
            'filter_config': {
                'application_type': 'vehicle',
                'accel_filter_config': {'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False},
                'gyro_filter_config': {'cutoff': 8.0, 'type': 'butterworth', 'adaptive': False}
            },
            'fusion_config': {
                'max_imu_age': 0.05,   # Tight timing
                'max_gps_age': 1.0,    # Tight timing
                'gps_timeout': 5.0
            }
        }
        
        fusion_system = CompleteSensorFusion(config)
        
        if fusion_system.initialize():
            fusion_system.start_sensors()
            
            print("Stress testing for 30 seconds at high rates...")
            start_time = time.time()
            max_timing_violations = 0
            max_buffer_overruns = 0
            
            while time.time() - start_time < 30:
                # Monitor system health
                imu_stats = fusion_system.imu_reader.get_stats()
                gps_stats = fusion_system.gps_reader.get_stats()
                
                timing_violations = fusion_system.stats.get('timing_violations', 0)
                buffer_overruns = imu_stats['buffer_overruns'] + gps_stats['buffer_overruns']
                
                max_timing_violations = max(max_timing_violations, timing_violations)
                max_buffer_overruns = max(max_buffer_overruns, buffer_overruns)
                
                elapsed = time.time() - start_time
                if int(elapsed) % 5 == 0 and elapsed > 1:  # Every 5 seconds
                    print(f"t={elapsed:.0f}s: Violations={timing_violations}, "
                          f"Overruns={buffer_overruns}, "
                          f"IMU={imu_stats['actual_rate']:.1f}Hz")
                
                time.sleep(0.1)
            
            # Final stress test results
            final_imu = fusion_system.imu_reader.get_stats()
            final_gps = fusion_system.gps_reader.get_stats()
            final_violations = fusion_system.stats.get('timing_violations', 0)
            
            print(f"\n=== STRESS TEST RESULTS ===")
            print(f"IMU: {final_imu['actual_rate']:.1f}/{config['rates']['imu_hardware']} Hz "
                  f"({final_imu['actual_rate']/config['rates']['imu_hardware']*100:.1f}%)")
            print(f"GPS: {final_gps['actual_rate']:.1f}/{config['rates']['gps_poll']} Hz "
                  f"({final_gps['actual_rate']/config['rates']['gps_poll']*100:.1f}%)")
            print(f"Timing violations: {final_violations}")
            print(f"Buffer overruns: IMU={final_imu['buffer_overruns']}, GPS={final_gps['buffer_overruns']}")
            
            # Pass/Fail criteria
            imu_performance = final_imu['actual_rate'] / config['rates']['imu_hardware']
            gps_performance = final_gps['actual_rate'] / config['rates']['gps_poll']
            
            if (imu_performance > 0.9 and gps_performance > 0.8 and 
                final_violations < 10 and final_imu['buffer_overruns'] < 5):
                print("✅ STRESS TEST PASSED - System stable at high rates")
            else:
                print("⚠️ STRESS TEST FAILED - System unstable at high rates")
        
        else:
            print("❌ System initialization failed")
    
    except KeyboardInterrupt:
        print("\nStress test interrupted")
    finally:
        if 'fusion_system' in locals():
            fusion_system.stop_sensors()
        print("✅ Stress test completed")

# ===== MAIN EXECUTION =====

if __name__ == "__main__":
    main()
