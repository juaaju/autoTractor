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
from ekfnparam3 import EKFSensorFusion  # Import EKF Anda
from mpu9250read import mpu9250
from mpu6050read import mpu6050

# GPSHandler dan IMUHandler sama seperti kode Anda, tapi dengan perbaikan kecil
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

class IMUHandler:
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
                print(f"IMU Exception: {e}")
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