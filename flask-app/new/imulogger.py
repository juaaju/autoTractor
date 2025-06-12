from mpu6050read import mpu6050
from mpu9250read import mpu9250
import csv
import time
import os
from datetime import datetime
import threading
from collections import deque

class HighSpeedMPULogger:
    def __init__(self, sensor_type='6050', target_hz=100, log_folder="mpu_logs"):
        self.sensor_type = sensor_type
        self.target_hz = target_hz
        self.target_interval = 1.0 / target_hz
        self.data_buffer = deque(maxlen=1000)  # Buffer untuk batch writing
        self.running = False
        
        # Setup folder untuk log
        self.log_folder = log_folder
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
            print(f"Created log folder: {self.log_folder}")
        
        # Initialize sensor
        if sensor_type == '6050':
            self.mpu = mpu6050(0x68)
            self.fieldnames = ['timestamp_unix', 'timestamp_readable', 'temp_c', 'accel_x', 'accel_y', 'accel_z', 
                             'gyro_x', 'gyro_y', 'gyro_z']
        else:
            self.mpu = mpu9250(0x68)
            #self.mpu.calibrate()
            self.fieldnames = ['timestamp_unix', 'timestamp_readable', 'temp_c', 'accel_x', 'accel_y', 'accel_z', 
                             'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'heading']
    
    def read_sensor_data_fast(self):
        """Optimized sensor reading without prints"""
        timestamp_unix = time.time()
        timestamp_readable = datetime.fromtimestamp(timestamp_unix).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        if self.sensor_type == '6050':
            # Read all data in one go
            temp = self.mpu.get_temp()
            accel_data = self.mpu.get_accel_data()
            gyro_data = self.mpu.get_gyro_data()
            
            return {
                'timestamp_unix': timestamp_unix,
                'timestamp_readable': timestamp_readable,
                'temp_c': temp,
                'accel_x': accel_data['x'],
                'accel_y': accel_data['y'],
                'accel_z': accel_data['z'],
                'gyro_x': gyro_data['x'],
                'gyro_y': gyro_data['y'],
                'gyro_z': gyro_data['z']
            }
        else:
            data = self.mpu.get_all_data()
            mag_x = mag_y = mag_z = None
            if data['mag'] is not None:
                mag_x, mag_y, mag_z = data['mag']['x'], data['mag']['y'], data['mag']['z']
            
            return {
                'timestamp_unix': timestamp_unix,
                'timestamp_readable': timestamp_readable,
                'temp_c': data['temp'],
                'accel_x': data['accel']['x'],
                'accel_y': data['accel']['y'],
                'accel_z': data['accel']['z'],
                'gyro_x': data['gyro']['x'],
                'gyro_y': data['gyro']['y'],
                'gyro_z': data['gyro']['z'],
                'mag_x': mag_x,
                'mag_y': mag_y,
                'mag_z': mag_z,
                'heading': data['heading'] if data['heading'] is not None else None
            }
    
    def data_acquisition_thread(self):
        """High-speed data acquisition thread"""
        print(f"Starting {self.target_hz}Hz data acquisition...")
        sample_count = 0
        start_time = time.time()
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Read sensor data
                data = self.read_sensor_data_fast()
                
                # Add to buffer
                self.data_buffer.append(data)
                sample_count += 1
                
                # Calculate actual sampling rate every 100 samples
                if sample_count % 100 == 0:
                    elapsed = time.time() - start_time
                    actual_hz = sample_count / elapsed
                    print(f"Samples: {sample_count}, Actual rate: {actual_hz:.1f} Hz")
                
                # Precise timing control
                elapsed = time.time() - loop_start
                sleep_time = max(0, self.target_interval - elapsed)
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                print(f"Error in acquisition: {e}")
                time.sleep(0.001)
    
    def file_writer_thread(self, filename):
        """Separate thread for writing data to file"""
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writeheader()
            
            write_count = 0
            while self.running or len(self.data_buffer) > 0:
                # Batch write untuk efisiensi
                batch_data = []
                while len(self.data_buffer) > 0 and len(batch_data) < 50:
                    data = self.data_buffer.popleft()
                    batch_data.append(data)
                
                if batch_data:
                    writer.writerows(batch_data)
                    csvfile.flush()  # Force write to disk
                    write_count += len(batch_data)
                    
                    if write_count % 100 == 0:
                        print(f"Written {write_count} samples to file")
                
                time.sleep(0.1)  # Write every 100ms
    
    def start_logging(self):
        """Start high-speed logging"""
        filename = f"mpu{self.sensor_type}_fast_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath = os.path.join(self.log_folder, filename)
        
        self.running = True
        
        # Start threads
        acq_thread = threading.Thread(target=self.data_acquisition_thread)
        writer_thread = threading.Thread(target=self.file_writer_thread, args=(filepath,))
        
        acq_thread.start()
        writer_thread.start()
        
        print(f"High-speed logging to {filepath}")
        print("Press Ctrl+C to stop...")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping...")
            self.running = False
            
            acq_thread.join()
            writer_thread.join()
            
            print(f"Data saved to {filepath}")
            print("Logging stopped")

def read_save_optimized():
    """Optimized version without threading (simpler)"""
    print("Pilih sensor:")
    print("1. MPU6050 (Accelerometer + Gyroscope)")
    print("2. MPU9250 (Accelerometer + Gyroscope + Magnetometer)")
    
    choice = input("Masukkan pilihan (1 atau 2): ")
    target_hz = int(input("Target sampling rate (Hz, default 100): ") or "100")
    
    # Setup folder untuk log
    log_folder = "mpu_logs"
    if not os.path.exists(log_folder):
        os.makedirs(log_folder)
        print(f"Created log folder: {log_folder}")
    
    if choice == '1':
        sensor_type = '6050'
        mpu = mpu6050(0x68)
        fieldnames = ['timestamp_unix', 'timestamp_readable', 'temp_c', 'accel_x', 'accel_y', 'accel_z', 
                     'gyro_x', 'gyro_y', 'gyro_z']
    elif choice == '2':
        sensor_type = '9250'
        mpu = mpu9250(0x68)
        #mpu.calibrate()
        fieldnames = ['timestamp_unix', 'timestamp_readable', 'temp_c', 'accel_x', 'accel_y', 'accel_z', 
                     'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'heading']
    else:
        print("Pilihan tidak valid")
        return
    
    filename = f"mpu{sensor_type}_optimized_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    filepath = os.path.join(log_folder, filename)
    target_interval = 1.0 / target_hz
    
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        print(f"Target: {target_hz} Hz, Saving to {filepath}")
        print("Press Ctrl+C to stop...")
        
        sample_count = 0
        start_time = time.time()
        
        try:
            while True:
                loop_start = time.time()
                
                # Generate timestamps
                timestamp_unix = time.time()
                timestamp_readable = datetime.fromtimestamp(timestamp_unix).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                
                # Fast data reading
                if sensor_type == '6050':
                    temp = mpu.get_temp()
                    accel_data = mpu.get_accel_data()
                    gyro_data = mpu.get_gyro_data()
                    
                    row_data = {
                        'timestamp_unix': timestamp_unix,
                        'timestamp_readable': timestamp_readable,
                        'temp_c': temp,
                        'accel_x': accel_data['x'],
                        'accel_y': accel_data['y'],
                        'accel_z': accel_data['z'],
                        'gyro_x': gyro_data['x'],
                        'gyro_y': gyro_data['y'],
                        'gyro_z': gyro_data['z']
                    }
                else:
                    data = mpu.get_all_data()
                    mag_x = mag_y = mag_z = None
                    if data['mag'] is not None:
                        mag_x, mag_y, mag_z = data['mag']['x'], data['mag']['y'], data['mag']['z']
                    
                    row_data = {
                        'timestamp_unix': timestamp_unix,
                        'timestamp_readable': timestamp_readable,
                        'temp_c': data['temp'],
                        'accel_x': data['accel']['x'],
                        'accel_y': data['accel']['y'],
                        'accel_z': data['accel']['z'],
                        'gyro_x': data['gyro']['x'],
                        'gyro_y': data['gyro']['y'],
                        'gyro_z': data['gyro']['z'],
                        'mag_x': mag_x,
                        'mag_y': mag_y,
                        'mag_z': mag_z,
                        'heading': data['heading'] if data['heading'] is not None else None
                    }
                
                writer.writerow(row_data)
                sample_count += 1
                
                # Print status setiap 100 samples (bukan setiap sample!)
                if sample_count % 100 == 0:
                    elapsed = time.time() - start_time
                    actual_hz = sample_count / elapsed
                    print(f"Samples: {sample_count}, Actual rate: {actual_hz:.1f} Hz")
                
                # Precise timing
                elapsed = time.time() - loop_start
                sleep_time = max(0, target_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            elapsed = time.time() - start_time
            actual_hz = sample_count / elapsed
            print(f"\nFinal rate: {actual_hz:.1f} Hz ({sample_count} samples in {elapsed:.1f}s)")
            print(f"Data saved to {filepath}")

def main():
    print("Pilih mode:")
    print("1. Optimized Simple (tanpa threading)")
    print("2. High-Speed Multi-threaded")
    print("3. Original (untuk perbandingan)")
    
    mode = input("Pilih mode (1/2/3): ")
    
    if mode == '1':
        read_save_optimized()
    elif mode == '2':
        print("Pilih sensor:")
        print("1. MPU6050")
        print("2. MPU9250")
        sensor_choice = input("Pilih (1/2): ")
        target_hz = int(input("Target Hz (default 100): ") or "100")
        
        sensor_type = '6050' if sensor_choice == '1' else '9250'
        logger = HighSpeedMPULogger(sensor_type, target_hz, log_folder="mpu_logs")
        logger.start_logging()
    else:
        print("Mode original - gunakan kode asli Anda")

if __name__ == "__main__":
    main()