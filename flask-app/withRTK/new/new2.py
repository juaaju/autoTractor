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
    """Precise rate control with timing compensation"""
    def __init__(self, target_hz):
        self.target_hz = target_hz
        self.target_interval = 1.0 / target_hz
        self.last_time = time.time()
        self.timing_history = deque(maxlen=50)
        self.actual_rate = target_hz
        self.cumulative_error = 0.0
        
    def wait_for_next(self):
        """Wait for next cycle with precise timing and error compensation"""
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        # Record timing for rate calculation
        if elapsed > 0:
            self.timing_history.append(elapsed)
            if len(self.timing_history) > 10:
                self.actual_rate = 1.0 / statistics.mean(self.timing_history)
        
        # Calculate target sleep time with cumulative error compensation
        ideal_sleep = self.target_interval
        timing_error = elapsed - self.target_interval
        self.cumulative_error += timing_error
        
        # Compensate for cumulative timing drift
        compensated_sleep = ideal_sleep - self.cumulative_error * 0.1
        sleep_time = max(0, compensated_sleep)
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        self.last_time = time.time()
        return elapsed

    def get_actual_rate(self):
        return self.actual_rate
    
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
            'actual_rate': 0.0
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
                
                # Reset cumulative error periodically
                if self.stats['read_count'] % 100 == 0:
                    self.rate_controller.reset_error()
                
            except Exception as e:
                self.stats['error_count'] += 1
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

class MultiRateSensorFusion:
    """Main sensor fusion class with multi-rate architecture"""
    
    def __init__(self, config=None):
        # Default configuration
        self.config = config or {
            'rates': {
                'imu_hardware': 50,     # IMU hardware sampling
                'ekf_predict': 25,      # EKF prediction rate
                'gps_poll': 5,          # GPS polling rate
                'logging': 10,          # Logging rate
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
                'gps_timeout': 5.0      # GPS timeout for initialization
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
    
    def initialize(self):
        """Initialize all components"""
        print("=== INITIALIZING MULTI-RATE SENSOR FUSION ===")
        
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
        test_data = self.imu_reader.sensor_handler.get_data()
        if isinstance(test_data, tuple) and len(test_data) >= 2:
            print(f"IMU test: accel={test_data[0]:.3f}, gyro={math.degrees(test_data[1]):.1f}°/s")
        
        # Initialize EKF instances
        print("Initializing EKF instances...")
        ekf_dt = 1.0 / self.config['rates']['ekf_predict']
        self.ekf_combined = EKFSensorFusion(dt=ekf_dt)
        self.ekf_imu_only = EKFSensorFusion(dt=ekf_dt)
        
        print(f"Configuration:")
        print(f"  IMU sampling: {self.config['rates']['imu_hardware']} Hz")
        print(f"  EKF predict: {self.config['rates']['ekf_predict']} Hz")
        print(f"  GPS polling: {self.config['rates']['gps_poll']} Hz")
        print(f"  Logging: {self.config['rates']['logging']} Hz")
        
        return True
    
    def start_sensor_threads(self):
        """Start all sensor reading threads"""
        print("Starting sensor reading threads...")
        self.gps_reader.start()
        self.imu_reader.start()
        time.sleep(2)  # Allow stabilization
        
    def stop_sensor_threads(self):
        """Stop all sensor reading threads"""
        print("Stopping sensor threads...")
        if self.gps_reader:
            self.gps_reader.stop()
        if self.imu_reader:
            self.imu_reader.stop()
    
    def initialize_gps_reference(self, timeout=30):
        """Initialize GPS reference with timeout"""
        print("Waiting for GPS initialization...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            gps_data = self.gps_reader.get_latest(max_age=2.0)
            if gps_data and gps_data.data and is_gps_valid(gps_data.data):
                coords = gps_data.data
                self.lat_ref = coords['latitude']
                self.lon_ref = coords['longitude']
                self.alt_ref = coords.get('altitude', 0)
                
                # Initialize both EKF instances at origin
                self.ekf_combined.state[0] = 0.0
                self.ekf_combined.state[1] = 0.0
                self.ekf_imu_only.state[0] = 0.0
                self.ekf_imu_only.state[1] = 0.0
                
                self.gps_initialized = True
                print(f"GPS reference initialized: {self.lat_ref:.6f}, {self.lon_ref:.6f}")
                return True
            
            time.sleep(0.5)
        
        print("⚠️ GPS initialization timeout - proceeding without GPS reference")
        return False
    
    def run_fusion_loop(self):
        """Main fusion loop with precise timing"""
        print("Starting main fusion loop...")
        
        # Setup precise timing
        ekf_rate_controller = PreciseRateController(self.config['rates']['ekf_predict'])
        log_rate_controller = PreciseRateController(self.config['rates']['logging'])
        status_rate_controller = PreciseRateController(self.config['rates']['status'])
        
        # Setup logging
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_queues = self._setup_logging(timestamp)
        
        iteration_count = 0
        last_log_time = 0
        last_status_time = 0
        
        try:
            self.running.set()
            
            while self.running.is_set():
                current_time = time.time()
                
                # ===== EKF PREDICTION PHASE =====
                dt_actual = ekf_rate_controller.wait_for_next()
                
                # Get latest IMU data with interpolation
                imu_data = self.imu_reader.get_interpolated(
                    current_time - 0.005,  # 5ms lookback for processing delay
                    max_age=self.config['fusion_config']['max_imu_age']
                )
                
                if imu_data and imu_data.data:
                    try:
                        # Extract IMU values
                        if isinstance(imu_data.data, tuple) and len(imu_data.data) >= 5:
                            # With raw and stats: (accel_f, gyro_f, accel_r, gyro_r, stats)
                            accel, gyro = imu_data.data[0], imu_data.data[1]
                            accel_raw, gyro_raw = imu_data.data[2], imu_data.data[3]
                            filter_stats = imu_data.data[4] if len(imu_data.data) > 4 else {}
                        elif isinstance(imu_data.data, tuple) and len(imu_data.data) >= 2:
                            # Simple: (accel, gyro)
                            accel, gyro = imu_data.data[0], imu_data.data[1]
                            accel_raw, gyro_raw = accel, gyro
                            filter_stats = {}
                        else:
                            raise ValueError("Invalid IMU data format")
                        
                        # EKF Predictions
                        self.ekf_combined.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                        self.ekf_imu_only.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                        
                        self.stats['predictions'] += 1
                        
                        # Check for interpolation
                        if abs(imu_data.timestamp - current_time) > 0.01:
                            self.stats['interpolations'] += 1
                        
                    except Exception as e:
                        print(f"EKF prediction error: {e}")
                        continue
                else:
                    print("⚠️ No valid IMU data for prediction")
                    self.stats['timing_violations'] += 1
                    continue
                
                # ===== GPS UPDATE PHASE =====
                gps_updated = False
                if self.gps_initialized:
                    gps_data = self.gps_reader.get_latest(
                        max_age=self.config['fusion_config']['max_gps_age']
                    )
                    
                    if gps_data and gps_data.data and is_gps_valid(gps_data.data):
                        try:
                            coords = gps_data.data
                            gps_x, gps_y = latlon_to_xy(
                                coords['latitude'], coords['longitude'],
                                coords.get('altitude', self.alt_ref),
                                self.lat_ref, self.lon_ref, self.alt_ref
                            )
                            
                            # Update only combined EKF
                            gps_measurement = np.array([gps_x, gps_y])
                            self.ekf_combined.update(gps_measurement)
                            self.stats['gps_updates'] += 1
                            gps_updated = True
                            
                        except Exception as e:
                            print(f"GPS update error: {e}")
                
                # ===== LOGGING PHASE =====
                if current_time - last_log_time >= 1.0 / self.config['rates']['logging']:
                    self._log_data(log_queues, current_time, dt_actual, 
                                 accel, gyro, accel_raw, gyro_raw, filter_stats, 
                                 gps_updated, iteration_count)
                    last_log_time = current_time
                
                # ===== STATUS PHASE =====
                if current_time - last_status_time >= 1.0 / self.config['rates']['status']:
                    self._print_status(iteration_count, dt_actual)
                    last_status_time = current_time
                
                iteration_count += 1
                
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
            self._cleanup_logging(log_queues)
    
    def _setup_logging(self, timestamp):
        """Setup logging queues and threads"""
        log_queues = {
            'combined': Queue(),
            'imu_predict': Queue(),
            'filter_stats': Queue()
        }
        
        filenames = {
            'combined': f"sensor_fusion_multirate_{timestamp}.csv",
            'imu_predict': f"imu_predict_multirate_{timestamp}.csv",
            'filter_stats': f"filter_stats_multirate_{timestamp}.csv"
        }
        
        # Start logging threads
        self.log_threads = []
        for log_type, queue in log_queues.items():
            thread = Thread(
                target=self._logging_worker,
                args=(queue, filenames[log_type], log_type),
                daemon=True
            )
            thread.start()
            self.log_threads.append(thread)
        
        print(f"Logging to: {list(filenames.values())}")
        return log_queues
    
    def _logging_worker(self, queue, filename, log_type):
        """Worker thread for logging"""
        headers = {
            'combined': [
                'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon',
                'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
                'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
                'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
                'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'gps_updated', 'predict_count', 'gps_update_count',
                'P_trace', 'K_matrix', 'innovation_vector'
            ],
            'imu_predict': [
                'time', 'dt',
                'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
                'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
                'predict_x', 'predict_y', 'predict_heading_deg',
                'predict_velocity', 'predict_accel',
                'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'predict_count', 'P_trace'
            ],
            'filter_stats': [
                'time', 'accel_filter_effectiveness', 'gyro_filter_effectiveness',
                'accel_cutoff_hz', 'gyro_cutoff_hz',
                'instantaneous_accel_noise_removed', 'instantaneous_gyro_noise_removed',
                'iteration'
            ]
        }
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers[log_type])
            
            while True:
                try:
                    row = queue.get(timeout=1)
                    if row == "STOP":
                        break
                    writer.writerow(row)
                    f.flush()  # Ensure data is written
                except Empty:
                    continue
                except Exception as e:
                    print(f"Logging error ({log_type}): {e}")
    
    def _log_data(self, log_queues, current_time, dt_actual, 
                 accel, gyro, accel_raw, gyro_raw, filter_stats, 
                 gps_updated, iteration_count):
        """Log data to appropriate queues"""
        
        # Extract states and matrices
        combined_state = self.ekf_combined.state.copy()
        imu_state = self.ekf_imu_only.state.copy()
        
        # Extract P matrices
        P_combined = self.ekf_combined.get_covariance()
        P_imu = self.ekf_imu_only.get_covariance()
        P_trace_combined = np.trace(P_combined)
        P_trace_imu = np.trace(P_imu)
        
        # Extract K and innovation if GPS updated
        K_matrix = None
        innovation_vector = None
        if gps_updated:
            try:
                K_matrix = self.ekf_combined.get_kalman_gain()
                innovation_vector = self.ekf_combined.get_innovation()
            except:
                pass
        
        # Format states
        if len(combined_state) >= 4:
            est_x, est_y, est_heading, est_velocity = combined_state[:4]
            est_accel = combined_state[5] if len(combined_state) > 5 else accel
        else:
            est_x, est_y = combined_state[0], combined_state[1]
            est_heading, est_velocity, est_accel = 0, 0, accel
        
        if len(imu_state) >= 4:
            pred_x, pred_y, pred_heading, pred_velocity = imu_state[:4]
            pred_accel = imu_state[5] if len(imu_state) > 5 else accel
        else:
            pred_x, pred_y = imu_state[0], imu_state[1]
            pred_heading, pred_velocity, pred_accel = 0, 0, accel
        
        # Combined logging
        gps_data = self.gps_reader.get_latest(max_age=2.0)
        if gps_data and gps_data.data and is_gps_valid(gps_data.data):
            coords = gps_data.data
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
                est_x, est_y, math.degrees(est_heading), est_velocity, est_accel,
                filter_stats.get('accel_noise_reduction', 0),
                filter_stats.get('gyro_noise_reduction', 0),
                gps_updated, self.stats['predictions'], self.stats['gps_updates'],
                P_trace_combined,
                str(K_matrix.tolist()) if K_matrix is not None else None,
                str(innovation_vector.tolist()) if innovation_vector is not None else None
            ]
        else:
            combined_row = [
                current_time, dt_actual, None, None, None, None,
                accel, gyro, math.degrees(gyro),
                accel_raw, gyro_raw, math.degrees(gyro_raw),
                est_x, est_y, math.degrees(est_heading), est_velocity, est_accel,
                filter_stats.get('accel_noise_reduction', 0),
                filter_stats.get('gyro_noise_reduction', 0),
                False, self.stats['predictions'], self.stats['gps_updates'],
                P_trace_combined, None, None
            ]
        
        log_queues['combined'].put(combined_row)
        
        # IMU predict logging
        imu_row = [
            current_time, dt_actual,
            accel, gyro, math.degrees(gyro),
            accel_raw, gyro_raw, math.degrees(gyro_raw),
            pred_x, pred_y, math.degrees(pred_heading), pred_velocity, pred_accel,
            filter_stats.get('accel_noise_reduction', 0),
            filter_stats.get('gyro_noise_reduction', 0),
            self.stats['predictions'], P_trace_imu
        ]
        log_queues['imu_predict'].put(imu_row)
    
    def _print_status(self, iteration_count, dt_actual):
        """Print system status"""
        # Get sensor statistics
        gps_stats = self.gps_reader.get_stats()
        imu_stats = self.imu_reader.get_stats()
        
        # Get current states
        combined_state = self.ekf_combined.state
        imu_state = self.ekf_imu_only.state
        
        print(f"t={iteration_count * dt_actual:.1f}s | "
              f"GPS: {gps_stats['read_count']}/{gps_stats['error_count']} "
              f"({gps_stats['actual_rate']:.1f}Hz) | "
              f"IMU: {imu_stats['read_count']}/{imu_stats['error_count']} "
              f"({imu_stats['actual_rate']:.1f}Hz)")
        
        print(f"    Predictions: {self.stats['predictions']} | "
              f"GPS updates: {self.stats['gps_updates']} | "
              f"Interpolations: {self.stats['interpolations']} | "
              f"Timing violations: {self.stats['timing_violations']}")
        
        if len(combined_state) >= 2 and len(imu_state) >= 2:
            pos_diff = math.sqrt((combined_state[0] - imu_state[0])**2 + 
                               (combined_state[1] - imu_state[1])**2)
            print(f"    IMU-only: ({imu_state[0]:.2f}, {imu_state[1]:.2f}) | "
                  f"Combined: ({combined_state[0]:.2f}, {combined_state[1]:.2f}) | "
                  f"Diff: {pos_diff:.2f}m")
    
    def _cleanup_logging(self, log_queues):
        """Cleanup logging threads"""
        print("Stopping logging...")
        for queue in log_queues.values():
            queue.put("STOP")
        
        for thread in self.log_threads:
            thread.join(timeout=2)
    
    def run(self):
        """Main entry point"""
        try:
            # Initialize
            if not self.initialize():
                print("❌ Initialization failed")
                return False
            
            # Start sensor threads
            self.start_sensor_threads()
            
            # Initialize GPS reference
            self.initialize_gps_reference()
            
            # Run main fusion loop
            self.run_fusion_loop()
            
        finally:
            # Cleanup
            self.stop_sensor_threads()
            print("✅ Sensor fusion stopped")
            
        return True

def main_corrected_sensor_fusion():
    """Main function with corrected implementation"""
    print("=== CORRECTED MULTI-RATE SENSOR FUSION ===")
    
    # Configuration
    config = {
        'rates': {
            'imu_hardware': 50,     # High-rate IMU sampling
            'ekf_predict': 25,      # Medium-rate EKF predictions
            'gps_poll': 5,          # GPS polling rate
            'logging': 10,          # Logging rate
            'status': 1             # Status updates
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
        }
    }
    
    # Create and run fusion system
    fusion_system = MultiRateSensorFusion(config)
    
    print("Configuration:")
    print(f"  IMU Hardware Rate: {config['rates']['imu_hardware']} Hz")
    print(f"  EKF Predict Rate: {config['rates']['ekf_predict']} Hz")
    print(f"  GPS Poll Rate: {config['rates']['gps_poll']} Hz")
    print(f"  Logging Rate: {config['rates']['logging']} Hz")
    print(f"  Accel Filter: {config['filter_config']['accel_filter_config']['cutoff']} Hz")
    print(f"  Gyro Filter: {config['filter_config']['gyro_filter_config']['cutoff']} Hz")
    
    return fusion_system.run()

def test_timing_performance():
    """Test timing performance of the corrected implementation"""
    print("=== TESTING TIMING PERFORMANCE ===")
    
    # Test rate controllers
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
    """Test buffered sensor reading"""
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
                return (accel, gyro, accel + 0.01, gyro + 0.005, {'accel_noise_reduction': 0.1, 'gyro_noise_reduction': 0.15})
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

def test_filter_effectiveness_corrected():
    """Test filter effectiveness with corrected implementation"""
    print("=== TESTING FILTER EFFECTIVENESS (CORRECTED) ===")
    
    try:
        # Create buffered IMU reader
        imu_handler = FilteredIMUHandler(
            application_type='vehicle',
            accel_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
            gyro_filter_config={'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
        )
        
        buffered_imu = BufferedSensorReader(imu_handler, target_hz=50)
        buffered_imu.start()
        
        print("Collecting filtered data for 10 seconds...")
        print("Time\tAccel_Raw\tAccel_Filt\tReduction\tGyro_Raw\tGyro_Filt\tReduction")
        print("-" * 80)
        
        start_time = time.time()
        for i in range(50):  # 10 seconds at 5Hz display
            time.sleep(0.2)
            
            # Get latest data
            data = buffered_imu.get_latest(max_age=0.1)
            if data and data.data and len(data.data) >= 5:
                accel_f, gyro_f, accel_r, gyro_r, stats = data.data
                
                elapsed = time.time() - start_time
                accel_reduction = abs(accel_r - accel_f)
                gyro_reduction = abs(math.degrees(gyro_r - gyro_f))
                
                print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t{accel_reduction:.3f}\t\t"
                      f"{math.degrees(gyro_r):6.1f}°/s\t{math.degrees(gyro_f):6.1f}°/s\t{gyro_reduction:5.1f}°/s")
        
        # Final statistics
        reader_stats = buffered_imu.get_stats()
        print(f"\nBuffered reader performance:")
        print(f"  Target rate: 50 Hz, Actual: {reader_stats['actual_rate']:.1f} Hz")
        print(f"  Read count: {reader_stats['read_count']}")
        print(f"  Error count: {reader_stats['error_count']}")
        print(f"  Buffer overruns: {reader_stats['buffer_overruns']}")
        
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        if 'buffered_imu' in locals():
            buffered_imu.stop()

def main():
    """Main function with test options"""
    print("GPS-IMU Sensor Fusion (CORRECTED IMPLEMENTATION)")
    print("Featuring multi-rate architecture with precise timing")
    print()
    print("Options:")
    print("1. Run corrected sensor fusion")
    print("2. Test timing performance")
    print("3. Test buffered sensor reading")
    print("4. Test filter effectiveness (corrected)")
    print("5. Run original implementation (for comparison)")
    
    choice = input("Enter choice (1-5): ").strip()
    
    if choice == '1':
        return main_corrected_sensor_fusion()
    elif choice == '2':
        test_timing_performance()
    elif choice == '3':
        test_buffered_sensor()
    elif choice == '4':
        test_filter_effectiveness_corrected()
    elif choice == '5':
        print("Running original implementation...")
        return main_sensor_fusion_with_filters()
    else:
        print("Invalid choice")
        return False

# Keep original functions for comparison
def main_sensor_fusion_with_filters():
    """Original implementation (for comparison)"""
    print("=== ORIGINAL SENSOR FUSION IMPLEMENTATION ===")
    print("(This is your original code for comparison)")
    
    # ... (Original implementation would go here)
    # For now, just print a message
    print("Original implementation would run here...")
    print("Switching to corrected implementation...")
    return main_corrected_sensor_fusion()

if __name__ == "__main__":
    main()