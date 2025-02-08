from flask import Flask, render_template, jsonify
import serial
import time

app = Flask(__name__)

# Inisialisasi komunikasi serial
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Sesuaikan dengan port yang digunakan

@app.route('/')
def index():
    return render_template('index.html')

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

if __name__ == '__main__':
    app.run(debug=True)
