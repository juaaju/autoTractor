from realtimefusion import DataCollector, RealTimeEKFSensorFusion
import json
import math
from flask import Flask, render_template, jsonify, request, make_response
import threading
import time
from flask_cors import CORS
import os
from pathplanning import generate_zigzag_path_90deg
import serial
import requests
from math import atan2, degrees  # Tambahkan ini
from autotraktor import auto_control

# Create Flask app
app = Flask(__name__)
CORS(app)

# Inisialisasi komunikasi serial
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Sesuaikan dengan port yang digunakan

# Global variables to store latest sensor data and log file path
latest_data = {}
log_file_path = ""
stored_data = {}  # Pastikan variabel ini didefinisikan secara global
auto_mode_enabled = False  # Tambahkan ini untuk kontrol mode otomatis

@app.route('/data_gps', methods=['GET'])
def gps_data():
    # Use dictionary access and provide defaults if keys don't exist
    return jsonify({
        "latitude": latest_data.get("gps_lat", None),
        "longitude": latest_data.get("gps_lng", None)
    })

@app.route('/data_imu', methods=['GET'])
def imu_data():
    return jsonify({
        "latitude": latest_data.get("imu_lat", None),
        "longitude": latest_data.get("imu_lng", None)
    })

@app.route('/data_ekf', methods=['GET'])
def ekf_data():
    return jsonify({
        "latitude": latest_data.get("ekf_lat", None),
        "longitude": latest_data.get("ekf_lng", None)
    })

@app.route('/data_sensor_all', methods=['GET'])
def get_data():
    """
    Endpoint to get the latest sensor data through a GET request.
    """
    return jsonify(latest_data)

@app.route('/log_file', methods=['GET'])
def get_log_file():
    """
    Endpoint to download the current sensor fusion log file.
    """
    global log_file_path
    if os.path.exists(log_file_path):
        return send_file(log_file_path, as_attachment=True)
    else:
        return jsonify({"error": "Log file not found"}), 404

def update_latest_data(sensor_data, x_imu, y_imu, imu_lat, imu_lng, theta_imu, ekf_x, ekf_y, ekf_lat, ekf_lng):
    """
    Update the global latest_data variable with new sensor readings.
    """
    global latest_data
    latest_data = {
        "gps_lat": sensor_data.gps_lat,
        "gps_lng": sensor_data.gps_lng,
        "imu_x": x_imu,
        "imu_y": y_imu,
        "imu_lat": imu_lat,
        "imu_lng": imu_lng,
        "imu_heading": math.degrees(theta_imu),
        "ekf_x": ekf_x,
        "ekf_y": ekf_y,
        "ekf_lat": ekf_lat,
        "ekf_lng": ekf_lng,
        "timestamp": time.time()
    }

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

def run_flask():
    """
    Run the Flask server in a separate thread.
    """
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)

def main():
    # Start Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True  # This ensures the thread will exit when the main program exits
    flask_thread.start()
    
    # Initialize data collector
    collector = DataCollector()
    
    # Initialize EKF with 250Hz update rate
    dt = 1/250.0
    fusion = RealTimeEKFSensorFusion(dt)
    
    # Store the log file path globally
    global log_file_path
    log_file_path = fusion.log_file_path

    # State awal untuk odometri IMU
    state = {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0, "theta": 0.0}
    
    # Variabel untuk kontrol otomatis
    current_path = None
    last_command_time = 0
    command_cooldown = 3.0  # waktu minimum antara perintah dalam detik

    try:
        print("Starting sensor fusion...")
        print("Data is now accessible via GET at http://ubuntu.local:5001/data_sensor")
        print("Log file is accessible via GET at http://ubuntu.local:5001/log_file")
        
        while True:
            # Get latest sensor data
            sensor_data = collector.get_latest_data()

            # Proses dengan EKF
            estimated_state = fusion.process_sensor_data(sensor_data)

            # Hitung posisi dengan odometri IMU
            x_imu, y_imu, theta_imu = odometry_imu(sensor_data, dt, state)

            imu_lat, imu_lon = local_to_geographic(x_imu, y_imu, ref_lat, ref_lon)
            ekf_lat, ekf_lon = local_to_geographic(estimated_state[0], estimated_state[1], ref_lat, ref_lon)

            # Update the latest data
            update_latest_data(sensor_data, x_imu, y_imu, imu_lat, imu_lon, theta_imu, 
                             estimated_state[0], estimated_state[1], ekf_lat, ekf_lon)
            
            # Cek apakah jalur tersedia
            global stored_data
            if stored_data and 'zigzag_path' in stored_data and stored_data['zigzag_path']:
                current_path = stored_data['zigzag_path']
            
            # Jika jalur tersedia dan mode otomatis diaktifkan, lakukan kontrol otomatis
            global auto_mode_enabled
            if current_path and auto_mode_enabled and (time.time() - last_command_time) >= command_cooldown:
                # Gunakan posisi dari EKF untuk kontrol
                current_position = (ekf_lat, ekf_lon)
                
                # Tentukan perintah kontrol
                command = auto_control(current_position, current_path)
                
                # Eksekusi perintah jika ada
                if command:
                    print(f"Executing auto command: {command}")
                    ser.write(command.encode())  # Kirim perintah ke Arduino
                    last_command_time = time.time()

    except KeyboardInterrupt:
        print("\nStopping sensor fusion...")
    finally:
        collector.stop()
        fusion.close()

#============================================================================================

#kontrol
@app.route('/turn_right', methods=['GET'])
def turn_right():
    ser.write(b'belok_kanan')  
    return jsonify({"status": "TURN RIGHT"})

@app.route('/turn_left', methods=['GET'])
def turn_left():
    ser.write(b'belok_kiri') 
    return jsonify({"status": "TURN LEFT"})

@app.route('/motor_off', methods=['GET'])
def motor_off():
    ser.write(b'stop')  # Mengirim perintah ke Arduino untuk mematikan LED
    return jsonify({"status": "MOTOR OFF"})

#================================ Untuk data inisialisasi ==============================
# Endpoint untuk menerima data inisialisasi dan proses path planning
@app.route('/initial_data', methods=['POST'])
def process_data():
    global stored_data  # Gunakan variabel global

    data = request.get_json()
    
    tractor_position = data.get('tractorPosition', {})
    field_points = data.get('fieldPoints', [])

    print("Received:", tractor_position, field_points)

    # Simulasi pemrosesan data / loading
    time.sleep(3)

    corners = [
        (float(field_points['point1']['latitude']), float(field_points['point1']['longitude'])),  
        (float(field_points['point2']['latitude']), float(field_points['point2']['longitude'])),
        (float(field_points['point3']['latitude']), float(field_points['point3']['longitude'])),
        (float(field_points['point4']['latitude']), float(field_points['point4']['longitude']))
    ]

    # Lebar alur pembajakan (meter)
    swath_width = 2.0
    
    # Menghasilkan jalur zigzag dengan sudut 90 derajat
    zigzag_path = generate_zigzag_path_90deg(corners, swath_width)
    print(zigzag_path)

    # Simpan data untuk diakses nanti
    stored_data = {
        "tractorPosition": tractor_position,
        "fieldPoints": field_points,
        "zigzag_path": zigzag_path
    }

    return jsonify({"message": "Path planning sudah dibuat kemudian data telah diproses dan disimpan."}), 200

# ======= Endpoint untuk mengirim initial data dan path planning ke React (GET) =======
@app.route('/data-form', methods=['GET'])
def send_data_path():
    global stored_data
    if not stored_data:
        return jsonify({"error": "Belum ada data"}), 404
    return jsonify(stored_data), 200

@app.route('/auto_mode', methods=['POST', 'OPTIONS'])
def toggle_auto_mode():
    # Menangani permintaan preflight OPTIONS
    if request.method == 'OPTIONS':
        return build_cors_preflight_response()
        
    global auto_mode_enabled
    data = request.get_json()
    
    if 'enabled' in data:
        auto_mode_enabled = data['enabled']
        return jsonify({"status": f"Auto mode {'enabled' if auto_mode_enabled else 'disabled'}"})
    
    return jsonify({"error": "Missing 'enabled' parameter"}), 400

def build_cors_preflight_response():
    response = make_response()
    response.headers.add("Access-Control-Allow-Origin", "*")
    response.headers.add("Access-Control-Allow-Headers", "Content-Type")
    response.headers.add("Access-Control-Allow-Methods", "POST")
    return response

if __name__ == "__main__":
    main()