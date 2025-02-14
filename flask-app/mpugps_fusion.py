import numpy as np
import math
import time
from typing import Tuple, Dict, Optional
from queue import Queue
from threading import Thread, Lock
import serial  # For reading sensor data
from dataclasses import dataclass
from datetime import datetime

@dataclass
class IMUData:
    timestamp: float
    acceleration: float
    angular_velocity: float

@dataclass
class GPSData:
    timestamp: float
    x: float
    y: float

class SensorReader:
    def __init__(self, imu_port: str, gps_port: str):
        self.imu_queue = Queue()
        self.gps_queue = Queue()
        self.last_gps_data: Optional[GPSData] = None
        self.running = True
        self.lock = Lock()

        # Initialize serial connections
        self.imu_serial = serial.Serial(imu_port, 115200)  # Adjust baud rate as needed
        self.gps_serial = serial.Serial(gps_port, 9600)    # Adjust baud rate as needed

        # Start reading threads
        self.imu_thread = Thread(target=self._read_imu_data)
        self.gps_thread = Thread(target=self._read_gps_data)
        self.imu_thread.start()
        self.gps_thread.start()

    def _read_imu_data(self):
        while self.running:
            if self.imu_serial.in_waiting:
                try:
                    # Modify this according to your IMU data format
                    raw_data = self.imu_serial.readline().decode().strip()
                    # Parse your IMU data format here
                    # This is an example parsing, adjust according to your sensor
                    accel, gyro = map(float, raw_data.split(','))
                    
                    imu_data = IMUData(
                        timestamp=time.time(),
                        acceleration=accel,
                        angular_velocity=gyro
                    )
                    self.imu_queue.put(imu_data)
                except Exception as e:
                    print(f"Error reading IMU data: {e}")

    def _read_gps_data(self):
        while self.running:
            if self.gps_serial.in_waiting:
                try:
                    # Modify this according to your GPS data format
                    raw_data = self.gps_serial.readline().decode().strip()
                    # Parse your GPS data format here
                    # This is an example parsing, adjust according to your sensor
                    x, y = map(float, raw_data.split(','))
                    
                    gps_data = GPSData(
                        timestamp=time.time(),
                        x=x,
                        y=y
                    )
                    with self.lock:
                        self.last_gps_data = gps_data
                    self.gps_queue.put(gps_data)
                except Exception as e:
                    print(f"Error reading GPS data: {e}")

    def get_synchronized_data(self) -> Tuple[IMUData, GPSData]:
        """Get synchronized IMU and GPS data at 250Hz"""
        imu_data = self.imu_queue.get()
        
        with self.lock:
            gps_data = self.last_gps_data

        if gps_data is None:
            # If no GPS data received yet, use zeros
            gps_data = GPSData(timestamp=imu_data.timestamp, x=0.0, y=0.0)

        return imu_data, gps_data

    def close(self):
        self.running = False
        self.imu_thread.join()
        self.gps_thread.join()
        self.imu_serial.close()
        self.gps_serial.close()

class RealTimeEKFSensorFusion:
    def __init__(self, dt: float):
        # Use the same EKF implementation from your original code
        self.ekf = EKFSensorFusion(dt)
        
        # Add data logging
        self.log_file = open(f"sensor_fusion_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
        self.log_file.write("timestamp,gps_x,gps_y,imu_accel,imu_gyro,estimated_x,estimated_y,estimated_heading\n")

    def process_sensor_data(self, imu_data: IMUData, gps_data: GPSData):
        # Prediction step
        self.ekf.predict()

        # Update step with measurements
        measurements = np.array([
            gps_data.x,
            gps_data.y,
            imu_data.acceleration,
            imu_data.angular_velocity
        ])
        
        self.ekf.update(measurements)

        # Log data
        estimated_state = self.ekf.state
        self.log_file.write(f"{imu_data.timestamp},{gps_data.x},{gps_data.y},"
                           f"{imu_data.acceleration},{imu_data.angular_velocity},"
                           f"{estimated_state[0]},{estimated_state[1]},{estimated_state[2]}\n")
        
        return estimated_state

    def close(self):
        self.log_file.close()

def main():
    # Initialize sensor readers with appropriate port names
    # Replace with your actual serial port names
    sensor_reader = SensorReader(
        imu_port="/dev/ttyUSB0",  # Adjust to your IMU port
        gps_port="/dev/ttyUSB1"   # Adjust to your GPS port
    )

    # Initialize EKF with 250Hz update rate
    dt = 1/250.0
    fusion = RealTimeEKFSensorFusion(dt)

    try:
        print("Starting sensor fusion...")
        while True:
            # Get synchronized sensor data
            imu_data, gps_data = sensor_reader.get_synchronized_data()
            
            # Process data through EKF
            estimated_state = fusion.process_sensor_data(imu_data, gps_data)
            
            # Print current state (optional)
            print(f"Position: ({estimated_state[0]:.2f}, {estimated_state[1]:.2f}), "
                  f"Heading: {math.degrees(estimated_state[2]):.1f}°")

    except KeyboardInterrupt:
        print("\nStopping sensor fusion...")
    finally:
        sensor_reader.close()
        fusion.close()

if __name__ == "__main__":
    main()