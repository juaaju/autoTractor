from flask import Flask, render_template, jsonify, request
import serial
import time
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

sensor_data = {"temperature": 0, "accel.x": 0}

# Inisialisasi komunikasi serial
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Sesuaikan dengan port yang digunakan

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

# proses data form
@app.route('/process_data', methods=['POST'])
def process_data():
    data = request.get_json()

    tractor_position = data['tractorPosition']
    field_points = data['fieldPoints']

    # Proses data
    time.sleep(5)  # Simulasi pengolahan data

    # Kembalikan respons setelah selesai
    return jsonify({"message": "Data telah diproses."}), 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
