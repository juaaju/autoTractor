import time
import math
import numpy as np
import statistics
from threading import Thread, Lock
from collections import deque
from gpsread import GPSReader
from mpu9250read import mpu9250
from mpu6050read import mpu6050
from lowpass_filters import LowPassFilter, AdaptiveLowPassFilter, design_filter_for_application

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
    
    def reset_error(self):
        """Reset cumulative error (call periodically)"""
        self.cumulative_error = 0.0

class GPSHandler:
    """GPS Handler dengan validation dan filtering (unchanged from original)"""
    def __init__(self, port="/dev/ttyUSB0"):
        self.reader = GPSReader(port)
        self.latest_coords = None
        self.prev_coords = None
        self.running = True
        self.lock = Lock()
        self.read_count = 0
        self.error_count = 0
        self.last_read_time = 0
        
        # Add rate controller for GPS
        self.rate_controller = PreciseRateController(10.0)  # 10Hz GPS
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        consecutive_errors = 0
        while self.running:
            try:
                dt = self.rate_controller.wait_for_next()
                
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
                            self.latest_coords = new_coords
                            self.prev_coords = self.latest_coords
                            self.read_count += 1
                            self.last_read_time = current_time
                        consecutive_errors = 0
                    else:
                        consecutive_errors += 1
                        self.error_count += 1
                        
            except Exception as e:
                self.error_count += 1
                print(f"GPS Exception: {e}")
                time.sleep(0.5)
    
    def validate_coordinates(self, coords):
        lat = coords['latitude']
        lon = coords['longitude']
        
        if abs(lat) > 90 or abs(lon) > 180:
            return False
        if lat == 0.0 and lon == 0.0:
            return False
            
        # Check for sudden jumps
        if self.prev_coords:
            distance = self.calculate_distance(self.prev_coords, coords)
            time_diff = coords['timestamp'] - self.prev_coords.get('timestamp', 0)
            if time_diff > 0 and distance / time_diff > 50:  # Max 50 m/s
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
            'last_read_time': self.last_read_time,
            'actual_rate': self.rate_controller.get_actual_rate(),
            'target_rate': 10.0,
            'timing_accuracy': self.rate_controller.get_timing_accuracy(),
            'timing_violations': self.rate_controller.get_timing_violations()
        }
    
    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        try:
            self.reader.close()
        except:
            pass

class FilteredIMUHandler:
    """
    Enhanced IMU Handler dengan integrated low-pass filtering dan proper rate monitoring
    """
    def __init__(self, i2c=0x68, 
                 accel_filter_config=None, 
                 gyro_filter_config=None,
                 application_type='vehicle',
                 target_rate=50.0):
        """
        Args:
            i2c: I2C address untuk IMU
            accel_filter_config: Dict dengan 'cutoff', 'type', 'adaptive'
            gyro_filter_config: Dict dengan 'cutoff', 'type', 'adaptive'
            application_type: 'vehicle', 'drone', 'walking_robot', dll
            target_rate: Target sampling rate dalam Hz
        """
        self.target_rate = target_rate
        
        # Initialize IMU hardware
        try:
            self.imu = mpu6050(i2c)
            self.imu.calibrate()
            print("✓ MPU9250 initialized and calibrated")
        except Exception as e:
            print(f"MPU9250 failed, trying MPU6050: {e}")
            try:
                self.imu = mpu6050(i2c)
                self.imu.calibrate()
                print("✓ MPU6050 initialized and calibrated")
            except Exception as e2:
                print(f"Both IMU initialization failed: {e2}")
                raise e2
        
        # Get recommended filter settings for application
        if accel_filter_config is None or gyro_filter_config is None:
            recommended = design_filter_for_application(application_type)
            print(f"Using recommended filter settings for '{application_type}':")
            print(f"  Accel cutoff: {recommended['accel_cutoff']} Hz")
            print(f"  Gyro cutoff: {recommended['gyro_cutoff']} Hz")
            print(f"  Filter type: {recommended['filter_type']}")
            print(f"  Adaptive: {recommended['adaptive']}")
        
        if accel_filter_config is None:
            accel_filter_config = {
                'cutoff': recommended['accel_cutoff'],
                'type': recommended['filter_type'],
                'adaptive': recommended['adaptive']
            }
        
        if gyro_filter_config is None:
            gyro_filter_config = {
                'cutoff': recommended['gyro_cutoff'],
                'type': recommended['filter_type'],
                'adaptive': recommended['adaptive']
            }
        
        # Initialize filters
        if accel_filter_config['adaptive']:
            self.accel_filter = AdaptiveLowPassFilter(
                accel_filter_config['cutoff'], target_rate
            )
            self.accel_adaptive = True
        else:
            self.accel_filter = LowPassFilter(
                accel_filter_config['cutoff'], 
                target_rate, 
                accel_filter_config['type']
            )
            self.accel_adaptive = False
        
        if gyro_filter_config['adaptive']:
            self.gyro_filter = AdaptiveLowPassFilter(
                gyro_filter_config['cutoff'], target_rate
            )
            self.gyro_adaptive = True
        else:
            self.gyro_filter = LowPassFilter(
                gyro_filter_config['cutoff'], 
                target_rate, 
                gyro_filter_config['type']
            )
            self.gyro_adaptive = False
        
        # State variables
        self.accel = 0
        self.gyro = 0
        self.accel_raw = 0
        self.gyro_raw = 0
        self.running = True
        self.lock = Lock()
        self.read_count = 0
        self.error_count = 0
        
        # Rate control
        self.rate_controller = PreciseRateController(target_rate)
        
        # Data history for proper filter effectiveness calculation
        self.history_size = max(200, int(target_rate * 4))  # 4 seconds of data
        self.accel_raw_history = deque(maxlen=self.history_size)
        self.accel_filtered_history = deque(maxlen=self.history_size)
        self.gyro_raw_history = deque(maxlen=self.history_size)
        self.gyro_filtered_history = deque(maxlen=self.history_size)
        
        # Filter effectiveness tracking
        self.filter_stats = {
            'accel_noise_reduction': 0,
            'gyro_noise_reduction': 0,
            'accel_cutoff': accel_filter_config['cutoff'],
            'gyro_cutoff': gyro_filter_config['cutoff'],
            'last_update_time': time.time()
        }
        
        # Start sensor reading thread
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
        
        print(f"✓ FilteredIMUHandler initialized with filters:")
        print(f"  Target rate: {target_rate} Hz")
        print(f"  Accel: {accel_filter_config}")
        print(f"  Gyro: {gyro_filter_config}")
        print(f"  History buffer: {self.history_size} samples")
    
    def update_loop(self):
        """Main sensor reading loop dengan proper rate control dan filtering"""
        while self.running:
            try:
                # Precise timing control
                dt = self.rate_controller.wait_for_next()
                
                # Read raw IMU data
                try:
                    data_accel = self.imu.get_accel_data()
                    data_gyro = self.imu.get_gyro_data()
                    
                    # Extract relevant axes (sesuaikan dengan kebutuhan Anda)
                    accel_raw = data_accel['x']*(-1)  # Forward acceleration (-1 khusus untuk mpu6050 karena pemasangannya terbalik)
                    gyro_raw = math.radians(data_gyro['z'])  # Yaw rate
                    
                except Exception as e:
                    print(f"IMU data read error: {e}")
                    accel_raw = 0
                    gyro_raw = 0
                    self.error_count += 1
                    continue
                
                # Apply filters
                try:
                    if self.accel_adaptive:
                        accel_filtered = self.accel_filter.filter_adaptive(accel_raw)
                    else:
                        accel_filtered = self.accel_filter.filter(accel_raw)
                    
                    if self.gyro_adaptive:
                        gyro_filtered = self.gyro_filter.filter_adaptive(gyro_raw)
                    else:
                        gyro_filtered = self.gyro_filter.filter(gyro_raw)
                
                except Exception as e:
                    print(f"Filter error: {e}")
                    accel_filtered = accel_raw
                    gyro_filtered = gyro_raw
                
                # Store data history for statistics
                self.accel_raw_history.append(accel_raw)
                self.accel_filtered_history.append(accel_filtered)
                self.gyro_raw_history.append(gyro_raw)
                self.gyro_filtered_history.append(gyro_filtered)
                
                # Calculate filter effectiveness every 2 seconds
                current_time = time.time()
                if (current_time - self.filter_stats['last_update_time'] > 2.0 and 
                    len(self.accel_raw_history) >= 100):
                    
                    self.update_filter_effectiveness()
                    self.filter_stats['last_update_time'] = current_time
                
                # Thread-safe update
                with self.lock:
                    self.accel = accel_filtered
                    self.gyro = gyro_filtered
                    self.accel_raw = accel_raw
                    self.gyro_raw = gyro_raw
                    self.read_count += 1
                
                # Reset cumulative timing error periodically
                if self.read_count % 200 == 0:
                    self.rate_controller.reset_error()
                    
            except Exception as e:
                self.error_count += 1
                print(f"FilteredIMU Exception: {e}")
                time.sleep(0.01)  # Short sleep to prevent tight error loop
    
    def update_filter_effectiveness(self):
        """Calculate proper filter effectiveness using variance reduction"""
        try:
            # Use sufficient data for reliable statistics
            min_samples = min(100, len(self.accel_raw_history))
            
            if min_samples < 50:
                return  # Not enough data yet
            
            # Get recent data
            accel_raw_recent = list(self.accel_raw_history)[-min_samples:]
            accel_filtered_recent = list(self.accel_filtered_history)[-min_samples:]
            gyro_raw_recent = list(self.gyro_raw_history)[-min_samples:]
            gyro_filtered_recent = list(self.gyro_filtered_history)[-min_samples:]
            
            # Calculate variances
            accel_raw_var = np.var(accel_raw_recent)
            accel_filtered_var = np.var(accel_filtered_recent)
            gyro_raw_var = np.var(gyro_raw_recent)
            gyro_filtered_var = np.var(gyro_filtered_recent)
            
            # Calculate noise reduction (proper formula)
            if accel_raw_var > 1e-6:  # Avoid division by zero
                accel_effectiveness = (accel_raw_var - accel_filtered_var) / accel_raw_var
                self.filter_stats['accel_noise_reduction'] = max(0, min(1, accel_effectiveness))
            
            if gyro_raw_var > 1e-6:  # Avoid division by zero
                gyro_effectiveness = (gyro_raw_var - gyro_filtered_var) / gyro_raw_var
                self.filter_stats['gyro_noise_reduction'] = max(0, min(1, gyro_effectiveness))
            
            # Update adaptive cutoff frequencies if applicable
            if self.accel_adaptive:
                self.filter_stats['accel_cutoff'] = self.accel_filter.get_current_cutoff()
            if self.gyro_adaptive:
                self.filter_stats['gyro_cutoff'] = self.gyro_filter.get_current_cutoff()
            
        except Exception as e:
            print(f"Filter effectiveness calculation error: {e}")
    
    def get_data(self, return_raw=False, return_stats=False):
        """
        Get filtered IMU data
        
        Args:
            return_raw: If True, return both filtered and raw data
            return_stats: If True, return filter statistics
            
        Returns:
            If return_raw=False: (accel_filtered, gyro_filtered)
            If return_raw=True: (accel_filtered, gyro_filtered, accel_raw, gyro_raw)
            If return_stats=True: Includes filter stats as third/fifth element
        """
        with self.lock:
            if return_raw and return_stats:
                return (self.accel, self.gyro, self.accel_raw, self.gyro_raw, 
                       self.filter_stats.copy())
            elif return_raw:
                return self.accel, self.gyro, self.accel_raw, self.gyro_raw
            elif return_stats:
                return self.accel, self.gyro, self.filter_stats.copy()
            else:
                return self.accel, self.gyro
    
    def get_stats(self):
        """Get comprehensive IMU and filter statistics"""
        with self.lock:
            total_attempts = self.read_count + self.error_count
            success_rate = self.read_count / total_attempts if total_attempts > 0 else 0
            
            stats = {
                'read_count': self.read_count,
                'error_count': self.error_count,
                'success_rate': success_rate,
                'actual_rate': self.rate_controller.get_actual_rate(),
                'target_rate': self.target_rate,
                'timing_accuracy': self.rate_controller.get_timing_accuracy(),
                'timing_violations': self.rate_controller.get_timing_violations(),
                'filter_effectiveness': self.filter_stats.copy(),
                'buffer_utilization': {
                    'accel_raw': len(self.accel_raw_history),
                    'accel_filtered': len(self.accel_filtered_history),
                    'max_size': self.history_size
                }
            }
        return stats
    
    def get_rate_info(self):
        """Get detailed rate information for monitoring"""
        return {
            'actual_hz': self.rate_controller.get_actual_rate(),
            'target_hz': self.target_rate,
            'timing_accuracy_percent': self.rate_controller.get_timing_accuracy() * 100,
            'timing_violations': self.rate_controller.get_timing_violations(),
            'samples_collected': self.read_count,
            'error_rate_percent': (self.error_count / max(1, self.read_count + self.error_count)) * 100
        }
    
    def reset_filters(self):
        """Reset all filters to initial state"""
        self.accel_filter.reset()
        self.gyro_filter.reset()
        
        # Clear history
        self.accel_raw_history.clear()
        self.accel_filtered_history.clear()
        self.gyro_raw_history.clear()
        self.gyro_filtered_history.clear()
        
        # Reset effectiveness stats
        self.filter_stats['accel_noise_reduction'] = 0
        self.filter_stats['gyro_noise_reduction'] = 0
        self.filter_stats['last_update_time'] = time.time()
        
        print("✓ IMU filters and statistics reset")
    
    def stop(self):
        """Stop IMU handler"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        print("✓ FilteredIMUHandler stopped")

class LegacyIMUHandler:
    """Legacy IMU Handler tanpa filtering (untuk compatibility) dengan rate monitoring"""
    def __init__(self, i2c=0x68, target_rate=50.0):
        self.target_rate = target_rate
        
        try:
            self.imu = mpu9250(i2c)
            self.imu.calibrate()
        except:
            self.imu = mpu6050(i2c)
            self.imu.calibrate()
            
        self.accel = 0
        self.gyro = 0
        self.running = True
        self.lock = Lock()
        self.read_count = 0
        self.error_count = 0
        
        # Add rate controller
        self.rate_controller = PreciseRateController(target_rate)
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            try:
                dt = self.rate_controller.wait_for_next()
                
                data_accel = self.imu.get_accel_data()
                data_gyro = self.imu.get_gyro_data()
                
                with self.lock:
                    self.accel = data_accel['x']
                    self.gyro = math.radians(data_gyro['z'])
                    self.read_count += 1
                    
            except Exception as e:
                self.error_count += 1
                print(f"Legacy IMU Exception: {e}")
                time.sleep(0.01)
    
    def get_data(self):
        with self.lock:
            return self.accel, self.gyro
    
    def get_stats(self):
        total_attempts = self.read_count + self.error_count
        success_rate = self.read_count / total_attempts if total_attempts > 0 else 0
        
        return {
            'read_count': self.read_count,
            'error_count': self.error_count,
            'success_rate': success_rate,
            'actual_rate': self.rate_controller.get_actual_rate(),
            'target_rate': self.target_rate,
            'timing_accuracy': self.rate_controller.get_timing_accuracy(),
            'timing_violations': self.rate_controller.get_timing_violations()
        }
    
    def get_rate_info(self):
        """Get detailed rate information for monitoring"""
        return {
            'actual_hz': self.rate_controller.get_actual_rate(),
            'target_hz': self.target_rate,
            'timing_accuracy_percent': self.rate_controller.get_timing_accuracy() * 100,
            'timing_violations': self.rate_controller.get_timing_violations(),
            'samples_collected': self.read_count,
            'error_rate_percent': (self.error_count / max(1, self.read_count + self.error_count)) * 100
        }
    
    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()

# Factory function untuk mudah switching
def create_imu_handler(use_filtering=True, target_rate=50.0, **kwargs):
    """
    Factory function untuk create IMU handler
    
    Args:
        use_filtering: True untuk FilteredIMUHandler, False untuk LegacyIMUHandler
        target_rate: Target sampling rate in Hz
        **kwargs: Arguments untuk handler (i2c, filter configs, etc.)
        
    Returns:
        IMU handler instance
    """
    if use_filtering:
        return FilteredIMUHandler(target_rate=target_rate, **kwargs)
    else:
        return LegacyIMUHandler(target_rate=target_rate, i2c=kwargs.get('i2c', 0x68))

# Enhanced test functions
def test_rate_monitoring(duration=30):
    """Test rate monitoring untuk melihat actual frequency"""
    print(f"=== Testing Rate Monitoring for {duration} seconds ===")
    
    target_rates = [25, 50, 100]  # Test different rates
    
    for target_rate in target_rates:
        print(f"\n--- Testing {target_rate} Hz Target Rate ---")
        
        try:
            imu_handler = FilteredIMUHandler(
                target_rate=target_rate,
                accel_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
                gyro_filter_config={'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
            )
            
            time.sleep(2)  # Stabilization
            
            print("Time\tActual_Hz\tTarget_Hz\tAccuracy\tSamples\tErrors\tEffectiveness")
            print("-" * 80)
            
            start_time = time.time()
            last_print_time = start_time
            
            while time.time() - start_time < duration:
                current_time = time.time()
                
                # Print status every 2 seconds
                if current_time - last_print_time >= 2.0:
                    rate_info = imu_handler.get_rate_info()
                    stats = imu_handler.get_stats()
                    
                    elapsed = current_time - start_time
                    
                    print(f"{elapsed:.1f}s\t{rate_info['actual_hz']:.1f}\t\t{rate_info['target_hz']:.0f}\t\t"
                          f"{rate_info['timing_accuracy_percent']:.1f}%\t\t{rate_info['samples_collected']}\t"
                          f"{stats['error_count']}\t{stats['filter_effectiveness']['accel_noise_reduction']:.1%}/"
                          f"{stats['filter_effectiveness']['gyro_noise_reduction']:.1%}")
                    
                    last_print_time = current_time
                
                time.sleep(0.1)
            
            # Final statistics
            final_stats = imu_handler.get_stats()
            final_rate = imu_handler.get_rate_info()
            
            print(f"\nFinal Results for {target_rate} Hz:")
            print(f"  Actual rate achieved: {final_rate['actual_hz']:.2f} Hz")
            print(f"  Timing accuracy: {final_rate['timing_accuracy_percent']:.1f}%")
            print(f"  Success rate: {final_stats['success_rate']:.1%}")
            print(f"  Timing violations: {final_rate['timing_violations']}")
            print(f"  Filter effectiveness: Accel={final_stats['filter_effectiveness']['accel_noise_reduction']:.1%}, "
                  f"Gyro={final_stats['filter_effectiveness']['gyro_noise_reduction']:.1%}")
            
            # Performance assessment
            rate_accuracy = abs(final_rate['actual_hz'] - target_rate) / target_rate
            if rate_accuracy < 0.05:  # Within 5%
                print(f"  ✅ EXCELLENT rate control (±{rate_accuracy*100:.1f}%)")
            elif rate_accuracy < 0.10:  # Within 10%
                print(f"  ✅ GOOD rate control (±{rate_accuracy*100:.1f}%)")
            else:
                print(f"  ⚠️ POOR rate control (±{rate_accuracy*100:.1f}%)")
            
            imu_handler.stop()
            time.sleep(1)
            
        except Exception as e:
            print(f"  Error testing {target_rate} Hz: {e}")

def test_filter_effectiveness_fixed(duration=20):
    """Test fixed filter effectiveness calculation"""
    print(f"=== Testing Fixed Filter Effectiveness for {duration} seconds ===")
    
    try:
        imu_handler = FilteredIMUHandler(
            target_rate=50,
            accel_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
            gyro_filter_config={'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
        )
        
        time.sleep(3)  # Let filters stabilize
        
        print("Time\tAccel_Raw\tAccel_Filt\tGyro_Raw\tGyro_Filt\tA_Eff\tG_Eff\tRate")
        print("-" * 90)
        
        start_time = time.time()
        last_print_time = start_time
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Print status every 2 seconds
            if current_time - last_print_time >= 2.0:
                # Get current data
                accel_f, gyro_f, accel_r, gyro_r, stats = imu_handler.get_data(
                    return_raw=True, return_stats=True
                )
                rate_info = imu_handler.get_rate_info()
                
                elapsed = current_time - start_time
                
                print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t"
                      f"{math.degrees(gyro_r):6.1f}°/s\t{math.degrees(gyro_f):6.1f}°/s\t"
                      f"{stats['accel_noise_reduction']:.1%}\t{stats['gyro_noise_reduction']:.1%}\t"
                      f"{rate_info['actual_hz']:.1f}Hz")
                
                last_print_time = current_time
            
            time.sleep(0.1)
        
        # Final comprehensive statistics
        final_stats = imu_handler.get_stats()
        final_rate = imu_handler.get_rate_info()
        
        print(f"\n=== FINAL COMPREHENSIVE RESULTS ===")
        print(f"Rate Performance:")
        print(f"  Target: {final_rate['target_hz']} Hz")
        print(f"  Actual: {final_rate['actual_hz']:.2f} Hz")
        print(f"  Accuracy: {final_rate['timing_accuracy_percent']:.1f}%")
        print(f"  Violations: {final_rate['timing_violations']}")
        print(f"  Error rate: {final_rate['error_rate_percent']:.1f}%")
        
        print(f"\nFilter Performance:")
        print(f"  Accelerometer effectiveness: {final_stats['filter_effectiveness']['accel_noise_reduction']:.1%}")
        print(f"  Gyroscope effectiveness: {final_stats['filter_effectiveness']['gyro_noise_reduction']:.1%}")
        print(f"  Accel cutoff: {final_stats['filter_effectiveness']['accel_cutoff']:.1f} Hz")
        print(f"  Gyro cutoff: {final_stats['filter_effectiveness']['gyro_cutoff']:.1f} Hz")
        
        print(f"\nData Quality:")
        print(f"  Total samples: {final_stats['read_count']}")
        print(f"  Success rate: {final_stats['success_rate']:.1%}")
        print(f"  Buffer utilization: {final_stats['buffer_utilization']['accel_raw']}/{final_stats['buffer_utilization']['max_size']}")
        
        # Effectiveness validation
        accel_eff = final_stats['filter_effectiveness']['accel_noise_reduction']
        gyro_eff = final_stats['filter_effectiveness']['gyro_noise_reduction']
        
        print(f"\nValidation:")
        if 0 <= accel_eff <= 0.5 and 0 <= gyro_eff <= 0.5:
            print("  ✅ Filter effectiveness values are REALISTIC (0-50%)")
        else:
            print("  ⚠️ Filter effectiveness values may be unrealistic")
            
        if final_rate['actual_hz'] > final_rate['target_hz'] * 0.95:
            print("  ✅ Rate control is STABLE")
        else:
            print("  ⚠️ Rate control has issues")
        
        imu_handler.stop()
        
    except Exception as e:
        print(f"Test error: {e}")
        import traceback
        traceback.print_exc()

def compare_filtered_vs_raw_enhanced():
    """Enhanced comparison between filtered vs raw IMU data"""
    print("=== Enhanced Comparison: Filtered vs Raw IMU Data ===")
    
    try:
        # Create both handlers with same target rate
        target_rate = 50
        
        filtered_imu = FilteredIMUHandler(
            target_rate=target_rate,
            application_type='vehicle'
        )
        raw_imu = LegacyIMUHandler(target_rate=target_rate)
        
        time.sleep(3)  # Stabilization
        
        print("Collecting data for 30 seconds...")
        print("Time\tFilt_A\t\tRaw_A\t\tDiff_A\t\tFilt_G\t\tRaw_G\t\tDiff_G\t\tF_Rate\tR_Rate")
        print("-" * 100)
        
        start_time = time.time()
        comparison_data = []
        
        for i in range(150):  # 30 seconds at 5Hz display
            # Get data from both
            accel_f, gyro_f = filtered_imu.get_data()
            accel_r, gyro_r = raw_imu.get_data()
            
            # Get rate info
            f_rate_info = filtered_imu.get_rate_info()
            r_rate_info = raw_imu.get_rate_info()
            
            elapsed = time.time() - start_time
            
            # Calculate differences
            accel_diff = abs(accel_f - accel_r)
            gyro_diff = abs(math.degrees(gyro_f - gyro_r))
            
            comparison_data.append({
                'time': elapsed,
                'accel_filtered': accel_f,
                'accel_raw': accel_r,
                'accel_diff': accel_diff,
                'gyro_filtered': math.degrees(gyro_f),
                'gyro_raw': math.degrees(gyro_r),
                'gyro_diff': gyro_diff,
                'filtered_rate': f_rate_info['actual_hz'],
                'raw_rate': r_rate_info['actual_hz']
            })
            
            if i % 25 == 0:  # Print every 5 seconds
                print(f"{elapsed:.1f}s\t{accel_f:.3f}\t\t{accel_r:.3f}\t\t{accel_diff:.3f}\t\t"
                      f"{math.degrees(gyro_f):6.1f}°/s\t{math.degrees(gyro_r):6.1f}°/s\t{gyro_diff:5.1f}°/s\t"
                      f"{f_rate_info['actual_hz']:.1f}Hz\t{r_rate_info['actual_hz']:.1f}Hz")
            
            time.sleep(0.2)
        
        # Analysis
        if len(comparison_data) > 10:
            accel_diffs = [d['accel_diff'] for d in comparison_data]
            gyro_diffs = [d['gyro_diff'] for d in comparison_data]
            
            # Statistics
            avg_accel_diff = statistics.mean(accel_diffs)
            avg_gyro_diff = statistics.mean(gyro_diffs)
            max_accel_diff = max(accel_diffs)
            max_gyro_diff = max(gyro_diffs)
            
            # Rate statistics
            f_rates = [d['filtered_rate'] for d in comparison_data]
            r_rates = [d['raw_rate'] for d in comparison_data]
            
            print(f"\n=== COMPARISON ANALYSIS ===")
            print(f"Filter Impact:")
            print(f"  Average accel difference: {avg_accel_diff:.4f} m/s²")
            print(f"  Average gyro difference: {avg_gyro_diff:.1f}°/s")
            print(f"  Maximum accel difference: {max_accel_diff:.4f} m/s²")
            print(f"  Maximum gyro difference: {max_gyro_diff:.1f}°/s")
            
            print(f"\nRate Performance:")
            print(f"  Filtered IMU avg rate: {statistics.mean(f_rates):.1f} Hz")
            print(f"  Raw IMU avg rate: {statistics.mean(r_rates):.1f} Hz")
            print(f"  Target rate: {target_rate} Hz")
            
            # Final statistics
            filtered_stats = filtered_imu.get_stats()
            raw_stats = raw_imu.get_stats()
            
            print(f"\nFinal Handler Statistics:")
            print(f"Filtered IMU:")
            print(f"  Reads: {filtered_stats['read_count']}, Errors: {filtered_stats['error_count']}")
            print(f"  Success rate: {filtered_stats['success_rate']:.1%}")
            print(f"  Timing accuracy: {filtered_stats['timing_accuracy']:.1%}")
            print(f"  Filter effectiveness: A={filtered_stats['filter_effectiveness']['accel_noise_reduction']:.1%}, "
                  f"G={filtered_stats['filter_effectiveness']['gyro_noise_reduction']:.1%}")
            
            print(f"Raw IMU:")
            print(f"  Reads: {raw_stats['read_count']}, Errors: {raw_stats['error_count']}")
            print(f"  Success rate: {raw_stats['success_rate']:.1%}")
            print(f"  Timing accuracy: {raw_stats['timing_accuracy']:.1%}")
        
        filtered_imu.stop()
        raw_imu.stop()
        
    except Exception as e:
        print(f"Comparison test error: {e}")
        import traceback
        traceback.print_exc()

def test_different_rates():
    """Test performance at different target rates"""
    print("=== Testing Performance at Different Target Rates ===")
    
    test_rates = [10, 25, 50, 75, 100]
    results = []
    
    for target_rate in test_rates:
        print(f"\n--- Testing {target_rate} Hz ---")
        
        try:
            imu_handler = FilteredIMUHandler(
                target_rate=target_rate,
                accel_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
                gyro_filter_config={'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
            )
            
            time.sleep(5)  # Stabilization and data collection
            
            # Get final statistics
            rate_info = imu_handler.get_rate_info()
            stats = imu_handler.get_stats()
            
            result = {
                'target_rate': target_rate,
                'actual_rate': rate_info['actual_hz'],
                'accuracy_percent': rate_info['timing_accuracy_percent'],
                'success_rate': stats['success_rate'],
                'timing_violations': rate_info['timing_violations'],
                'samples_collected': rate_info['samples_collected'],
                'filter_effectiveness': {
                    'accel': stats['filter_effectiveness']['accel_noise_reduction'],
                    'gyro': stats['filter_effectiveness']['gyro_noise_reduction']
                }
            }
            
            results.append(result)
            
            print(f"  Target: {target_rate} Hz")
            print(f"  Actual: {rate_info['actual_hz']:.1f} Hz")
            print(f"  Accuracy: {rate_info['timing_accuracy_percent']:.1f}%")
            print(f"  Success: {stats['success_rate']:.1%}")
            print(f"  Violations: {rate_info['timing_violations']}")
            print(f"  Filter eff: A={stats['filter_effectiveness']['accel_noise_reduction']:.1%}, "
                  f"G={stats['filter_effectiveness']['gyro_noise_reduction']:.1%}")
            
            imu_handler.stop()
            time.sleep(1)
            
        except Exception as e:
            print(f"  Error testing {target_rate} Hz: {e}")
    
    # Summary table
    if results:
        print(f"\n{'='*80}")
        print("RATE PERFORMANCE SUMMARY")
        print(f"{'='*80}")
        print(f"{'Target':<8} {'Actual':<8} {'Accuracy':<10} {'Success':<8} {'Violations':<12} {'Filter Eff':<15}")
        print("-" * 80)
        
        for result in results:
            rate_diff = abs(result['actual_rate'] - result['target_rate']) / result['target_rate'] * 100
            
            print(f"{result['target_rate']:<8} {result['actual_rate']:<8.1f} "
                  f"{result['accuracy_percent']:<10.1f}% {result['success_rate']:<8.1%} "
                  f"{result['timing_violations']:<12} "
                  f"A:{result['filter_effectiveness']['accel']:.0%}/G:{result['filter_effectiveness']['gyro']:.0%}")
        
        # Find optimal rate
        best_result = min(results, key=lambda x: abs(x['actual_rate'] - x['target_rate']) / x['target_rate'])
        print(f"\n🏆 BEST RATE CONTROL: {best_result['target_rate']} Hz "
              f"(±{abs(best_result['actual_rate'] - best_result['target_rate'])/best_result['target_rate']*100:.1f}%)")

if __name__ == "__main__":
    print("Enhanced Filtered GPS/IMU Handler Test Suite with Rate Monitoring")
    print("1. Test rate monitoring at different frequencies")
    print("2. Test fixed filter effectiveness calculation")
    print("3. Enhanced comparison: filtered vs raw IMU data")
    print("4. Test performance at different target rates")
    print("5. Test GPS handler (with rate monitoring)")
    
    choice = input("Enter choice (1-5): ").strip()
    
    if choice == '1':
        test_rate_monitoring(duration=20)
    elif choice == '2':
        test_filter_effectiveness_fixed(duration=15)
    elif choice == '3':
        compare_filtered_vs_raw_enhanced()
    elif choice == '4':
        test_different_rates()
    elif choice == '5':
        print("Testing GPS handler with rate monitoring...")
        gps = GPSHandler()
        time.sleep(5)
        
        print("Time\tLatitude\tLongitude\tRate\tAccuracy\tErrors")
        print("-" * 70)
        
        start_time = time.time()
        for i in range(30):  # 30 seconds
            coords = gps.get_coords()
            stats = gps.get_stats()
            elapsed = time.time() - start_time
            
            if coords:
                print(f"{elapsed:.1f}s\t{coords['latitude']:.6f}\t{coords['longitude']:.6f}\t"
                      f"{stats['actual_rate']:.1f}Hz\t{stats['timing_accuracy']:.1%}\t{stats['error_count']}")
            else:
                print(f"{elapsed:.1f}s\tNo data\t\tNo data\t\t"
                      f"{stats['actual_rate']:.1f}Hz\t{stats['timing_accuracy']:.1%}\t{stats['error_count']}")
            
            time.sleep(1)
        
        final_stats = gps.get_stats()
        print(f"\nFinal GPS Stats:")
        print(f"  Target rate: 10 Hz")
        print(f"  Actual rate: {final_stats['actual_rate']:.1f} Hz")
        print(f"  Timing accuracy: {final_stats['timing_accuracy']:.1%}")
        print(f"  Success: {final_stats['read_count']}/{final_stats['read_count'] + final_stats['error_count']}")
        
        gps.stop()
    else:
        print("Invalid choice")
        print("\nQuick demo - showing rate info:")
        try:
            imu = FilteredIMUHandler(target_rate=50)
            time.sleep(3)
            
            for i in range(10):
                rate_info = imu.get_rate_info()
                print(f"Sample {i+1}: {rate_info['actual_hz']:.1f} Hz "
                      f"(accuracy: {rate_info['timing_accuracy_percent']:.1f}%)")
                time.sleep(1)
            
            imu.stop()
        except Exception as e:
            print(f"Demo error: {e}")
