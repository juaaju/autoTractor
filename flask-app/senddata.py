from realtimefusion import DataCollector, RealTimeEKFSensorFusion
import math
import requests
import json

# Server Flask
FLASK_SERVER_URL = "http://localhost:5000/data_latlon"

def send_data_to_flask(sensor_data, x_imu, y_imu, theta_imu, ekf_x, ekf_y):
    """
    Mengirim data ke server Flask dalam format JSON.
    """
    data = {
        "gps_lat": sensor_data.gps_lat,
        "gps_lng": sensor_data.gps_lng,
        "imu_x": x_imu,
        "imu_y": y_imu,
        "imu_heading": math.degrees(theta_imu),
        "ekf_x": ekf_x,
        "ekf_y": ekf_y
    }

    try:
        response = requests.post(FLASK_SERVER_URL, json=data)
        print("Response from server:", response.text)
    except requests.exceptions.RequestException as e:
        print("Error sending data:", e)

# Konstanta
R = 6371000  # Jari-jari bumi dalam meter
# Titik referensi GPS
ref_lat = -6.1754  # Latitude referensi
ref_lon = 106.8272  # Longitude referensi

def local_to_geographic(x, y, ref_lat, ref_lon):
    """
    Mengonversi koordinat lokal (x, y) dalam meter ke latitude dan longitude.
    
    :param x: Koordinat x lokal (meter)
    :param y: Koordinat y lokal (meter)
    :param ref_lat: Latitude referensi (derajat)
    :param ref_lon: Longitude referensi (derajat)
    :return: Latitude dan longitude (derajat)
    """
    # Konversi ref_lat ke radian
    ref_lat_rad = math.radians(ref_lat)
    
    # Hitung latitude dan longitude baru
    lat = ref_lat + (y / R) * (180 / math.pi)
    lon = ref_lon + (x / (R * math.cos(ref_lat_rad))) * (180 / math.pi)
    
    return lat, lon

def odometry_imu(sensor_data, dt, state):
    """
    Menghitung posisi menggunakan odometri dari IMU (dead reckoning)
    """
    # Ambil data akselerasi dan gyro
    ax = sensor_data.imu_accel_x  # m/s²
    ay = sensor_data.imu_accel_y  # m/s²
    wz = sensor_data.imu_gyro_z   # rad/s

    # Update orientasi (theta) dengan gyro
    state["theta"] += wz * dt  # Integrasi kecepatan sudut untuk mendapatkan orientasi (heading)

    # Integrasi akselerasi untuk mendapatkan kecepatan
    state["vx"] += ax * dt
    state["vy"] += ay * dt

    # Integrasi kecepatan untuk mendapatkan posisi
    state["x"] += state["vx"] * dt
    state["y"] += state["vy"] * dt

    # Transformasi ke koordinat global
    x_global = state["x"] * math.cos(state["theta"]) - state["y"] * math.sin(state["theta"])
    y_global = state["x"] * math.sin(state["theta"]) + state["y"] * math.cos(state["theta"])

    return x_global, y_global, state["theta"]

def main():
    # Initialize data collector
    collector = DataCollector()
    
    # Initialize EKF with 250Hz update rate
    dt = 1/250.0
    fusion = RealTimeEKFSensorFusion(dt)

    # State awal untuk odometri IMU
    state = {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0, "theta": 0.0}

    try:
        print("Starting sensor fusion...")
        while True:
            # Get latest sensor data
            sensor_data = collector.get_latest_data()
            print(sensor_data)

            # Proses dengan EKF
            estimated_state = fusion.process_sensor_data(sensor_data)

            #ngambil info x, y gps
            gps_x, gps_y = fusion.gps_to_local_coordinates(sensor_data.gps_lat, sensor_data.gps_lng)

            # Hitung posisi dengan odometri IMU
            x_imu, y_imu, theta_imu = odometry_imu(sensor_data, dt, state)

            imu_lat, imu_lon = local_to_geographic(x_imu, y_imu, ref_lat, ref_lon)
            ekf_lat, ekf_lon = local_to_geographic(estimated_state[0], estimated_state[1], ref_lat, ref_lon)

            # Print hasil EKF dan odometri IMU
            print(f"Position EKF: ({estimated_state[0]:.2f}, {estimated_state[1]:.2f}), "
                  f"Heading: {math.degrees(estimated_state[2]):.1f}°"
                  f"Position GPS, x: {gps_x}, y: {gps_y}")
            
            print(f"Position IMU: ({x_imu:.2f}, {y_imu:.2f}), Heading: {math.degrees(theta_imu):.1f}°")

            print(f"LAT LON, gps_lat: {sensor_data.gps_lat,}, gps_lng: {sensor_data.gps_lng}, imu_lat: {imu_lat}, imu_lng: {imu_lon}, ekf_lat: {ekf_lat}, ekf_lon: {ekf_lon}")

            send_data_to_flask(sensor_data, x_imu, y_imu, theta_imu, estimated_state[0], estimated_state[1])

    except KeyboardInterrupt:
        print("\nStopping sensor fusion...")
    finally:
        collector.stop()
        fusion.close()

if __name__ == "__main__":
    main()
