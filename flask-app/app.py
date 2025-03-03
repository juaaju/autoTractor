from flask import Flask, render_template, jsonify, request
import serial
import requests
import time
from flask_cors import CORS
import random
from realtimefusion import DataCollector
from pathplanning import generate_zigzag_path_90deg

app = Flask(__name__)
CORS(app)

# Inisialisasi komunikasi serial
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Sesuaikan dengan port yang digunakan

@app.route('/')
def index():
    return "Server Flask Berjalan!"

@app.route('/turn_right', methods=['GET'])
def turn_right():
    ser.write(b'belok_kanan')  
    return jsonify({"status": "TURN RIGHT"})

@app.route('/turn_left', methods=['GET'])
def turn_left():
    ser.write(b'belok_kiri') 
    return jsonify({"status": "TURN LEFT"})

@app.route('/cw_right', methods=['GET'])
def cw_right():
    ser.write(b'cw_kanan')
    return jsonify({"status": "CW KANAN"})

@app.route('/ccw_right', methods=['GET'])
def ccw_right():
    ser.write(b'ccw_kanan') 
    return jsonify({"status": "CCW KANAN"})

@app.route('/cw_left', methods=['GET'])
def cw_left():
    ser.write(b'cw_kiri')
    return jsonify({"status": "CW KIRI"})

@app.route('/ccw_left', methods=['GET'])
def ccw_left():
    ser.write(b'ccw_kiri')  
    return jsonify({"status": "CCW KIRI"})

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


# ======================= Data dummy ====================
@app.route('/api/gps-dummy', methods=['GET'])
# =====untuk mengirim data gps==============
# Simulate database or sensor readings
def get_current_gps_data():
    # In real application, this would come from actual GPS sensors or database
    return {
        "latitude": 51.505 + random.uniform(-0.01, 0.01),
        "longitude": -0.09 + random.uniform(-0.01, 0.01)
    }

@app.route('/api/imu-dummy', methods=['GET'])
# =====untuk mengirim data imu==============
# Simulate database or sensor readings
def get_current_imu_data():
    # In real application, this would come from actual GPS sensors or database
    return {
        "latitude": 51.505 + random.uniform(-0.01, 0.01),
        "longitude": -0.09 + random.uniform(-0.01, 0.01)
    }

@app.route('/api/ekf-dummy', methods=['GET'])
# =====untuk mengirim data imu==============
# Simulate database or sensor readings
def get_current_ekf_data():
    # In real application, this would come from actual GPS sensors or database
    return {
        "latitude": 51.505 + random.uniform(-0.01, 0.01),
        "longitude": -0.09 + random.uniform(-0.01, 0.01)
    }

#=======================================================

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
