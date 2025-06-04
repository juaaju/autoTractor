"""
Enhanced GPS and IMU handlers dengan integrated low-pass filtering
File: filtered_gpsimuhandler.py
"""

import time
import math
import numpy as np
from threading import Thread, Lock
from gpsread import GPSReader
from mpu9250read import mpu9250
from mpu6050read import mpu6050
from lowpass_filters import LowPassFilter, AdaptiveLowPassFilter, design_filter_for_application

class GPSHandler:
    """
    GPS Handler dengan validation dan filtering
    (Sama seperti kode asli Anda, tidak perlu filtering untuk GPS)
    """
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

class FilteredIMUHandler:
    """
    Enhanced IMU Handler dengan integrated low-pass filtering
    untuk mengurangi noise dari getaran mesin
    """
    def __init__(self, i2c=0x68, 
                 accel_filter_config=None, 
                 gyro_filter_config=None,
                 application_type='vehicle'):
        """
        Args:
            i2c: I2C address untuk IMU
            accel_filter_config: Dict dengan 'cutoff', 'type', 'adaptive'
            gyro_filter_config: Dict dengan 'cutoff', 'type', 'adaptive'
            application_type: 'vehicle', 'drone', 'walking_robot', dll
        """
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
        sampling_freq = 50.0  # 50Hz IMU sampling
        
        if accel_filter_config['adaptive']:
            self.accel_filter = AdaptiveLowPassFilter(
                accel_filter_config['cutoff'], sampling_freq
            )
            self.accel_adaptive = True
        else:
            self.accel_filter = LowPassFilter(
                accel_filter_config['cutoff'], 
                sampling_freq, 
                accel_filter_config['type']
            )
            self.accel_adaptive = False
        
        if gyro_filter_config['adaptive']:
            self.gyro_filter = AdaptiveLowPassFilter(
                gyro_filter_config['cutoff'], sampling_freq
            )
            self.gyro_adaptive = True
        else:
            self.gyro_filter = LowPassFilter(
                gyro_filter_config['cutoff'], 
                sampling_freq, 
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
        
        # Filter effectiveness tracking
        self.filter_stats = {
            'accel_noise_reduction': 0,
            'gyro_noise_reduction': 0,
            'accel_cutoff': accel_filter_config['cutoff'],
            'gyro_cutoff': gyro_filter_config['cutoff']
        }
        
        # Start sensor reading thread
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
        
        print(f"✓ FilteredIMUHandler initialized with filters:")
        print(f"  Accel: {accel_filter_config}")
        print(f"  Gyro: {gyro_filter_config}")
    
    def update_loop(self):
        """Main sensor reading loop dengan filtering"""
        while self.running:
            try:
                time.sleep(0.02)  # 50Hz IMU
                
                # Read raw IMU data
                try:
                    data_accel = self.imu.get_accel_data()
                    data_gyro = self.imu.get_gyro_data()
                    
                    # Extract relevant axes (sesuaikan dengan kebutuhan Anda)
                    accel_raw = data_accel['x']  # Forward acceleration
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
                
                # Calculate filter effectiveness (setelah beberapa readings)
                if self.read_count > 20:  # Skip initial readings
                    accel_diff = abs(accel_raw - accel_filtered)
                    gyro_diff = abs(gyro_raw - gyro_filtered)
                    
                    # Calculate noise reduction percentage
                    accel_noise_reduction = accel_diff / (abs(accel_raw) + 1e-6)
                    gyro_noise_reduction = gyro_diff / (abs(gyro_raw) + 1e-6)
                    
                    # Moving average of effectiveness
                    alpha = 0.05  # Slow adaptation
                    self.filter_stats['accel_noise_reduction'] = (
                        alpha * accel_noise_reduction + 
                        (1-alpha) * self.filter_stats['accel_noise_reduction']
                    )
                    self.filter_stats['gyro_noise_reduction'] = (
                        alpha * gyro_noise_reduction + 
                        (1-alpha) * self.filter_stats['gyro_noise_reduction']
                    )
                    
                    # Update adaptive cutoff frequencies
                    if self.accel_adaptive:
                        self.filter_stats['accel_cutoff'] = self.accel_filter.get_current_cutoff()
                    if self.gyro_adaptive:
                        self.filter_stats['gyro_cutoff'] = self.gyro_filter.get_current_cutoff()
                
                # Thread-safe update
                with self.lock:
                    self.accel = accel_filtered
                    self.gyro = gyro_filtered
                    self.accel_raw = accel_raw
                    self.gyro_raw = gyro_raw
                    self.read_count += 1
                    
            except Exception as e:
                self.error_count += 1
                print(f"FilteredIMU Exception: {e}")
                time.sleep(0.1)
    
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
            stats = {
                'read_count': self.read_count,
                'error_count': self.error_count,
                'success_rate': self.read_count / (self.read_count + self.error_count) if (self.read_count + self.error_count) > 0 else 0,
                'filter_effectiveness': self.filter_stats.copy()
            }
        return stats
    
    def reset_filters(self):
        """Reset all filters to initial state"""
        self.accel_filter.reset()
        self.gyro_filter.reset()
        print("✓ IMU filters reset")
    
    def stop(self):
        """Stop IMU handler"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        print("✓ FilteredIMUHandler stopped")

class LegacyIMUHandler:
    """
    Legacy IMU Handler tanpa filtering (untuk compatibility)
    Sama seperti kode asli Anda
    """
    def __init__(self, i2c=0x68):
        self.imu = mpu9250(i2c)
        self.imu.calibrate()
        self.accel = 0
        self.gyro = 0
        self.running = True
        self.lock = Lock()
        self.read_count = 0
        self.error_count = 0
        
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def update_loop(self):
        while self.running:
            try:
                time.sleep(0.02)  # 50Hz IMU
                
                data_accel = self.imu.get_accel_data()
                data_gyro = self.imu.get_gyro_data()
                
                with self.lock:
                    self.accel = data_accel['x']
                    self.gyro = math.radians(data_gyro['z'])
                    self.read_count += 1
                    
            except Exception as e:
                self.error_count += 1
                print(f"Legacy IMU Exception: {e}")
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
        if self.thread.is_alive():
            self.thread.join()

# Factory function untuk mudah switching
def create_imu_handler(use_filtering=True, **kwargs):
    """
    Factory function untuk create IMU handler
    
    Args:
        use_filtering: True untuk FilteredIMUHandler, False untuk LegacyIMUHandler
        **kwargs: Arguments untuk handler (i2c, filter configs, etc.)
        
    Returns:
        IMU handler instance
    """
    if use_filtering:
        return FilteredIMUHandler(**kwargs)
    else:
        return LegacyIMUHandler(kwargs.get('i2c', 0x68))

# Test functions
def test_filtered_imu(duration=10):
    """Test filtered IMU untuk melihat efektivitas filter"""
    print(f"=== Testing Filtered IMU for {duration} seconds ===")
    
    # Test dengan different filter settings
    configs = {
        'Conservative': {
            'accel_filter_config': {'cutoff': 1.0, 'type': 'butterworth', 'adaptive': False},
            'gyro_filter_config': {'cutoff': 1.0, 'type': 'butterworth', 'adaptive': False}
        },
        'Vehicle': {
            'accel_filter_config': {'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
            'gyro_filter_config': {'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
        },
        'Adaptive': {
            'accel_filter_config': {'cutoff': 2.0, 'type': 'butterworth', 'adaptive': True},
            'gyro_filter_config': {'cutoff': 3.0, 'type': 'butterworth', 'adaptive': True}
        }
    }
    
    for config_name, config in configs.items():
        print(f"\n--- Testing {config_name} Configuration ---")
        
        try:
            imu_handler = FilteredIMUHandler(**config)
            time.sleep(2)  # Let it stabilize
            
            start_time = time.time()
            samples = []
            
            while time.time() - start_time < duration:
                accel_f, gyro_f, accel_r, gyro_r = imu_handler.get_data(return_raw=True)
                
                samples.append({
                    'time': time.time() - start_time,
                    'accel_filtered': accel_f,
                    'gyro_filtered': gyro_f,
                    'accel_raw': accel_r,
                    'gyro_raw': gyro_r
                })
                
                time.sleep(0.1)  # 10Hz sampling for test
            
            # Analyze results
            if len(samples) > 10:
                accel_raw_std = np.std([s['accel_raw'] for s in samples])
                accel_filtered_std = np.std([s['accel_filtered'] for s in samples])
                gyro_raw_std = np.std([s['gyro_raw'] for s in samples])
                gyro_filtered_std = np.std([s['gyro_filtered'] for s in samples])
                
                accel_noise_reduction = accel_raw_std / accel_filtered_std if accel_filtered_std > 0 else 1
                gyro_noise_reduction = gyro_raw_std / gyro_filtered_std if gyro_filtered_std > 0 else 1
                
                stats = imu_handler.get_stats()
                
                print(f"  Samples collected: {len(samples)}")
                print(f"  Accel noise reduction: {accel_noise_reduction:.2f}x")
                print(f"  Gyro noise reduction: {gyro_noise_reduction:.2f}x")
                print(f"  Read success rate: {stats['success_rate']:.1%}")
                print(f"  Filter effectiveness: Accel={stats['filter_effectiveness']['accel_noise_reduction']:.1%}, "
                      f"Gyro={stats['filter_effectiveness']['gyro_noise_reduction']:.1%}")
            
            imu_handler.stop()
            time.sleep(1)
            
        except Exception as e:
            print(f"  Error testing {config_name}: {e}")

def compare_filtered_vs_raw():
    """Compare filtered vs raw IMU data side by side"""
    print("=== Comparing Filtered vs Raw IMU Data ===")
    
    try:
        # Create both handlers
        filtered_imu = FilteredIMUHandler(application_type='vehicle')
        raw_imu = LegacyIMUHandler()
        
        time.sleep(2)  # Stabilization
        
        print("Collecting data for 30 seconds...")
        print("Time\tFiltered_A\tRaw_A\t\tFiltered_G\tRaw_G")
        print("-" * 60)
        
        start_time = time.time()
        
        for i in range(300):  # 30 seconds at 10Hz
            # Get data from both
            accel_f, gyro_f = filtered_imu.get_data()
            accel_r, gyro_r = raw_imu.get_data()
            
            elapsed = time.time() - start_time
            
            if i % 10 == 0:  # Print every second
                print(f"{elapsed:.1f}s\t{accel_f:.3f}\t\t{accel_r:.3f}\t\t"
                      f"{math.degrees(gyro_f):.1f}°/s\t\t{math.degrees(gyro_r):.1f}°/s")
            
            time.sleep(0.1)
        
        # Final statistics
        filtered_stats = filtered_imu.get_stats()
        raw_stats = raw_imu.get_stats()
        
        print(f"\nFinal Statistics:")
        print(f"Filtered IMU: {filtered_stats['read_count']} reads, {filtered_stats['error_count']} errors")
        print(f"Raw IMU: {raw_stats['read_count']} reads, {raw_stats['error_count']} errors")
        print(f"Filter effectiveness: {filtered_stats['filter_effectiveness']}")
        
        filtered_imu.stop()
        raw_imu.stop()
        
    except Exception as e:
        print(f"Comparison test error: {e}")

if __name__ == "__main__":
    print("Filtered GPS/IMU Handler Test Suite")
    print("1. Test filtered IMU with different configurations")
    print("2. Compare filtered vs raw IMU data")
    print("3. Test GPS handler (basic)")
    
    choice = input("Enter choice (1-3): ").strip()
    
    if choice == '1':
        test_filtered_imu(duration=15)
    elif choice == '2':
        compare_filtered_vs_raw()
    elif choice == '3':
        print("Testing GPS handler...")
        gps = GPSHandler()
        time.sleep(5)
        for i in range(20):
            coords = gps.get_coords()
            if coords:
                print(f"GPS #{i}: {coords['latitude']:.6f}, {coords['longitude']:.6f}")
            else:
                print(f"GPS #{i}: No data")
            time.sleep(1)
        gps.stop()
    else:
        print("Invalid choice")
