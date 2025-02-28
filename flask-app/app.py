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

# Variabel global untuk menyimpan data terbaru
latest_data = {}
sensor_data = {"temperature": 0, "accel.x": 0}

# Inisialisasi komunikasi serial
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Sesuaikan dengan port yang digunakan

@app.route('/')
def index():
    return "Server Flask Berjalan!"

@app.route('/turn_right', methods=['GET'])
def turn_right():
    ser.write(b'cw')  # Mengirim perintah ke Arduino untuk menyalakan LED
    return jsonify({"status": "TURN RIGHT"})

@app.route('/turn_left', methods=['GET'])
def turn_left():
    ser.write(b'ccw')  # Mengirim perintah ke Arduino untuk mematikan LED
    return jsonify({"status": "TURN LEFT"})

@app.route('/motor_off', methods=['GET'])
def motor_off():
    ser.write(b'stop')  # Mengirim perintah ke Arduino untuk mematikan LED
    return jsonify({"status": "MOTOR OFF"})

# untuk kebutuh grafik mpu6050
@app.route('/update_data', methods=['POST'])
def update_data():
    global sensor_data
    data = request.json
    if data:  # Pastikan data tidak kosong
        sensor_data = data
        return jsonify({"message": "Data updated successfully"}), 200
    return jsonify({"error": "Invalid data format"}), 400

@app.route('/get_data', methods=['GET'])
def get_data():
    return jsonify(sensor_data), 200

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

@app.route('/api/gps-data', methods=['GET'])
# =====untuk mengirim data gps==============
# Simulate database or sensor readings
def get_current_gps_data():
    # In real application, this would come from actual GPS sensors or database
    return {
        "latitude": 51.505 + random.uniform(-0.01, 0.01),
        "longitude": -0.09 + random.uniform(-0.01, 0.01)
    }

def gps_data():
    return jsonify(get_current_gps_data())

#===================================================
@app.route('/data_latlon', methods=['POST'])
def data_latlon():
    """
    Menerima data dari client dan menyimpannya.
    """
    global latest_data
    data = request.get_json()

    if data is None:
        return jsonify({"error": "Invalid JSON"}), 400

    print(f"Received data: {data}")

    # Simpan data terbaru
    latest_data = data

    return jsonify({"status": "success"}), 200

@app.route('/get_datalatlon', methods=['GET'])
def get_datalatlon():
    """
    Mengembalikan data terbaru untuk front-end.
    """
    return jsonify(latest_data)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
