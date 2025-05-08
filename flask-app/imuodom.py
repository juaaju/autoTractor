from mpu9250read import mpu9250
from mpu6050read import mpu6050
from dfrobotimu import IMU_WT61PCTTL
from time import sleep, time
import numpy as np
import math

# Kalibrasi IMU
samples = 0
gravity = 9.8
mpu = mpu9250(0x68)
mpu = IMU_WT61PCTTL(port='/dev/ttyTHS0', baudrate=9600)
gyro_z = []
acc_x = []
acc_y = []
acc_z = []

# Inisialisasi variabel global odometry
x, y, phi, v = 0.0, 0.0, 0.0, 0.0

def odometry_imu(a, omega_deg, dt):
    if -0.001<a <0.001:
        a=0
    global x, y, phi, v
    # Konversi omega dari deg/s ke rad/s
    omega_rad = omega_deg * (math.pi / 180)
    
    x = x + v * dt * math.cos(phi)
    y = y + v * dt * math.sin(phi)
    phi = phi + omega_rad * dt
    v = v + a * dt

print("Sedang kalibrasi IMU, mengambil sampel ...")
while samples < 100:
    try:
        data = mpu.get_all_data()
        gyro_z.append(data["gyro"]["z"])
        acc_x.append(data["accel"]["x"])
        acc_y.append(data["accel"]["y"])
        acc_z.append(data["accel"]["z"])
        samples += 1
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        break

print("Menghitung error rata-rata")
sleep(3)
# Hitung rata-rata dengan numpy
mean_gyro_z = np.mean(gyro_z)
mean_acc_x = np.mean(acc_x)
mean_acc_y = np.mean(acc_y)
mean_acc_z = np.mean(acc_z)
print(f"Mean error acc x,y,z: {mean_acc_x:.3f}, {mean_acc_y:.3f}, {mean_acc_z:.3f}")
print(f"Mean error gyro z: {mean_gyro_z:.3f}")
sleep(3)

print("====DATA RAW DAN ODOMETRY====")
last_print_time = 0
print_interval = 0.001  # Interval print dalam detik

# Variabel untuk mengukur waktu sebenarnya antara pembacaan IMU
last_imu_time = time()

while True:
    try:
        # Pengambilan data tetap real-time tanpa delay
        data = mpu.get_all_data()
        
        # Hitung dt yang sebenarnya
        current_time = time()
        dt = current_time - last_imu_time  # dt sebenarnya
        last_imu_time = current_time  # Update waktu terakhir
        
        accel_x = data['accel']['x'] - mean_acc_x
        # accel_y = data['accel']['y'] - mean_acc_y
        # accel_z = data['accel']['z'] - mean_acc_z + gravity
        gyros_z = data['gyro']['z'] - mean_gyro_z
        
        # Panggil fungsi odometry dengan dt yang sebenarnya
        odometry_imu(accel_x, gyros_z, dt)
        
        # Cek apakah sudah waktunya untuk update tampilan
        if current_time - last_print_time >= print_interval:
            # Gabungkan semua output dalam satu print statement
            output = f"Raw: Temp: {data['temp']:.2f}°C | Accel X: {accel_x:.2f} m/s² | Gyro Z: {gyros_z:.2f} deg/s | dt: {dt:.4f}s | "
            output += f"Odom: x: {x:.2f}m | y: {y:.2f}m | phi: {phi:.2f}rad | v: {v:.2f}m/s"
            
            # Cetak semua dalam satu baris
            print(output, end='\r')
            
            # Update waktu print terakhir
            last_print_time = current_time
        
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        break
    except Exception as e:
        print(f"Error: {e}")
        sleep(0.1)  # Sedikit delay jika terjadi error