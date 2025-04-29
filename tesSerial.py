import serial
import time

# Tentukan port serial yang digunakan (ganti dengan port yang sesuai)
port = "/dev/ttyUSB0"  # Untuk Windows, bisa juga '/dev/ttyUSB0' di Linux
baudrate = 9600

# Membuka port serial
ser = serial.Serial(port, baudrate)

# Tunggu agar serial terkoneksi dengan Arduino
time.sleep(2)  # Waktu tunggu untuk memastikan komunikasi serial stabil

# Baca dan cetak pesan dari Arduino
while True:
    if ser.in_waiting > 0:
        message = ser.readline().decode('utf-8').strip()  # Membaca pesan dari Arduino
        print("Pesan dari Arduino:", message)
