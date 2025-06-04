from mpu6050read import mpu6050
from mpu9250read import mpu9250
import csv
import time
from datetime import datetime

def read_save6050():
    """Read and save MPU6050 data to CSV file"""
    mpu = mpu6050(0x68)
    
    # Simple calibration - just call this once
    mpu.calibrate()
    
    # Create CSV file with timestamp
    filename = f"mpu6050_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'temp_c', 'accel_x', 'accel_y', 'accel_z', 
                     'gyro_x', 'gyro_y', 'gyro_z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        print(f"Saving MPU6050 data to {filename}")
        print("Press Ctrl+C to stop...")
        
        try:
            while True:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                temp = mpu.get_temp()
                accel_data = mpu.get_accel_data()
                gyro_data = mpu.get_gyro_data()
                
                # Print to console
                print(f"Time: {timestamp}")
                print(f"Temperature (C): {temp}")
                print(f"Acceleration x (m/s^2): {accel_data['x']}")
                print(f"Acceleration y (m/s^2): {accel_data['y']}")
                print(f"Acceleration z (m/s^2): {accel_data['z']}")
                print(f"Gyroscope x (deg/s): {gyro_data['x']}")
                print(f"Gyroscope y (deg/s): {gyro_data['y']}")
                print(f"Gyroscope z (deg/s): {gyro_data['z']}")
                print("----------------------------------------")
                
                # Write to CSV
                writer.writerow({
                    'timestamp': timestamp,
                    'temp_c': temp,
                    'accel_x': accel_data['x'],
                    'accel_y': accel_data['y'],
                    'accel_z': accel_data['z'],
                    'gyro_x': gyro_data['x'],
                    'gyro_y': gyro_data['y'],
                    'gyro_z': gyro_data['z']
                })
                
                time.sleep(0.01)  # 100Hz
                
        except KeyboardInterrupt:
            print(f"\nData saved to {filename}")
            print("Program terminated by user")

def read_save9250():
    """Read and save MPU9250 data to CSV file"""
    mpu = mpu9250(0x68)
    mpu.calibrate()
    
    # Create CSV file with timestamp
    filename = f"mpu9250_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'temp_c', 'accel_x', 'accel_y', 'accel_z', 
                     'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'heading']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        print(f"Saving MPU9250 data to {filename}")
        print("Press Ctrl+C to stop...")
        
        try:
            while True:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                data = mpu.get_all_data()
                
                # Print to console
                print(f"Time: {timestamp}")
                print(f"Temperature (C): {data['temp']}")
                print(f"Acceleration x (m/s^2): {data['accel']['x']}")
                print(f"Acceleration y (m/s^2): {data['accel']['y']}")
                print(f"Acceleration z (m/s^2): {data['accel']['z']}")
                print(f"Gyroscope x (deg/s): {data['gyro']['x']}")
                print(f"Gyroscope y (deg/s): {data['gyro']['y']}")
                print(f"Gyroscope z (deg/s): {data['gyro']['z']}")
                
                mag_x = mag_y = mag_z = None
                if data['mag'] is not None:
                    mag_x, mag_y, mag_z = data['mag']['x'], data['mag']['y'], data['mag']['z']
                    print(f"Magnetometer x (µT): {mag_x}")
                    print(f"Magnetometer y (µT): {mag_y}")
                    print(f"Magnetometer z (µT): {mag_z}")
                
                heading = data['heading'] if data['heading'] is not None else None
                if heading is not None:
                    print(f"Compass Heading: {heading}° from North")
                
                print("----------------------------------------")
                
                # Write to CSV
                writer.writerow({
                    'timestamp': timestamp,
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
                    'heading': heading
                })
                
                time.sleep(0.01)  # 100Hz
                
        except KeyboardInterrupt:
            print(f"\nData saved to {filename}")
            print("Program terminated by user")
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.01)

def main():
    print("Pilih sensor:")
    print("1. MPU6050 (Accelerometer + Gyroscope)")
    print("2. MPU9250 (Accelerometer + Gyroscope + Magnetometer)")
    
    try:
        choice = int(input("Masukkan pilihan (1 atau 2): "))
        
        if choice == 1:
            print("Memulai pembacaan MPU6050...")
            read_save6050()
        elif choice == 2:
            print("Memulai pembacaan MPU9250...")
            read_save9250()
        else:
            print("Pilihan tidak valid. Silakan pilih 1 atau 2.")
            
    except ValueError:
        print("Input tidak valid. Silakan masukkan angka 1 atau 2.")
    except Exception as e:
        print(f"Terjadi kesalahan: {e}")

if __name__ == "__main__":
    main()