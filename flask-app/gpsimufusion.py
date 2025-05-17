import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread
from queue import Queue
from gpsread import GPSReader
from dfrobotimu import IMU_WT61PCTTL
import pymap3d as pm
from ekfnparam import EKFSensorFusion  # sesuaikan path modul EKF-mu

class GPSHandler:
    def __init__(self, port="/dev/ttyUSB0"):
        self.reader = GPSReader(port)
        self.latest_coords = None
        self.running = True
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()

    def update_loop(self):
        while self.running:
            coords = self.reader.read()
            if coords:
                self.latest_coords = coords

    def get_coords(self):
        return self.latest_coords

    def stop(self):
        self.running = False
        self.thread.join()
        self.reader.close()

class IMUHandler:
    def __init__(self, port="/dev/ttyTHS0"):
        self.imu = IMU_WT61PCTTL(port)
        self.imu.calibrate_bias(samples=200)
        self.accel = 0
        self.gyro = 0
        self.running = True
        self.thread = Thread(target=self.update_loop)
        self.thread.daemon = True
        self.thread.start()

    def update_loop(self):
        while self.running:
            data = self.imu.get_all_data(unit='m/s2')
            self.accel = data['accel']['x']
            self.gyro = math.radians(data['gyro']['z'])  # konversi langsung

    def get_data(self):
        return self.accel, self.gyro

    def stop(self):
        self.running = False
        self.thread.join()

def simple_latlon2xy(lat, lon, lat0, lon0):
    """
    Konversi koordinat geografis (lat, lon) menjadi koordinat kartesian (x, y) dalam meter
    relatif terhadap titik referensi (lat0, lon0)
    """
    R = 6371000  # Radius bumi dalam meter
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    # x positif ke arah timur
    x = R * dlon * math.cos(math.radians(lat0))
    # y positif ke arah utara
    y = R * dlat
    return x, y

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
    return e, n  # east (x), north (y)

def logging_thread_func(q: Queue, filename: str):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'gps_x', 'gps_y', 'lat', 'lon', 'accel', 'gyro', 'est_x', 'est_y', 'est_heading_deg'])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

# Inisialisasi
ekf = EKFSensorFusion(dt=0.05)
gps_handler = GPSHandler()
imu_handler = IMUHandler()
log_queue = Queue()
csv_filename = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

logging_thread = Thread(target=logging_thread_func, args=(log_queue, csv_filename))
logging_thread.start()

gps_x, gps_y = 0.0, 0.0  # atau nilai default yang aman
# Definisikan referensi di luar loop
reference_lat, reference_lon = None, None
first_gps = True  # Flag untuk mendeteksi GPS pertama
last_prediction_time = time.time()
# Parameter untuk adaptasi bias
POS_CORRECTION_THRESHOLD = 3.0  # meter, sesuaikan berdasarkan akurasi GPS Anda
ACCEL_BIAS_LEARNING_RATE = 0.005  # faktor pembelajaran untuk koreksi bias

try:
    while True:
        current_time = time.time()
        dt = current_time - last_prediction_time  # Hitung dt aktual
        last_prediction_time = current_time
        
        # Update dt di EKF (asumsi ada metode atau atribut untuk ini)
        ekf.dt = dt  # Jika EKF memiliki atribut dt
        # Atau ekf.update_dt(dt)  # Jika EKF memiliki metode untuk mengupdate dt
        accel, gyro = imu_handler.get_data()
        ekf.predict(imu_accel=accel, imu_omega=gyro)
        
        coords = gps_handler.get_coords()

        if coords:
            lat, lon = coords
#            print(f"Raw GPS: ({lat:.6f}, {lon:.6f})")
            
            # Set titik referensi pada pembacaan GPS pertama
            if reference_lat is None:
                reference_lat, reference_lon = lat, lon
                print(f"Set reference point: {reference_lat}, {reference_lon}")
            
            # Konversi ke XY berdasarkan titik referensi yang tetap
            gps_x, gps_y = latlon_to_xy(lat, lon, 0, reference_lat, reference_lon, 0)
            # print(f"GPS XY: ({gps_x:.2f}, {gps_y:.2f})")
            
            # Reset state EKF pada pembacaan GPS pertama
            if first_gps:
                ekf.state[0] = gps_x
                ekf.state[1] = gps_y
                ekf.state[4] = 0.0  # Reset kecepatan
                first_gps = False
                print("Initialized EKF state with first GPS reading")
            
            # Update EKF dengan measurement GPS
            measurement = np.array([gps_x, gps_y])
            ekf.update(measurement)
            
            # Sekarang, SETELAH update, lakukan adaptasi bias
            pos_correction = np.linalg.norm(measurement - np.array([ekf.state[0], ekf.state[1]]))
            if pos_correction > POS_CORRECTION_THRESHOLD:
                # Update bias dinamis
                accel_correction = -ACCEL_BIAS_LEARNING_RATE * ekf.state[4]
                imu_handler.imu.accel_bias['x'] += accel_correction
                print(f"Updated accel bias: {imu_handler.imu.accel_bias['x']:.6f}")

        # Tampilkan kecepatan estimasi setiap iterasi
        speed = ekf.state[4]
        
        # Tampilkan posisi estimasi
        x, y, phi = ekf.state[0], ekf.state[1], ekf.state[2]
        heading_deg = math.degrees(phi)
        print(f"Est Speed: {speed:.4f} m/s, GPSxy: ({gps_x:.2f}, {gps_y:.2f}), Est Pos: ({x:.4f}, {y:.4f}), Heading: {heading_deg:.3f}°", end="\r")
        
        # Log data
        if coords:
            log_queue.put([time.time(), gps_x, gps_y, coords[0], coords[1], accel, gyro, x, y, heading_deg])
        else:
            log_queue.put([time.time(), None, None, None, None, accel, gyro, x, y, heading_deg])

except KeyboardInterrupt:
    print("Stopping...")
    gps_handler.stop()
    imu_handler.stop()
    log_queue.put("STOP")
    logging_thread.join()
