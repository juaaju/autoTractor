import serial

# Ganti port dan baudrate sesuai GPS kamu
PORT = '/dev/ttyUSB0'         # Contoh untuk Windows, atau '/dev/ttyUSB0' di Linux
BAUDRATE = 115200   # Umumnya 9600 atau 115200 untuk Quectel

try:
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        print(f"Terhubung ke {PORT} pada {BAUDRATE} bps\nMenunggu data NMEA...\n")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$'):
                print("NMEA:", line)
except serial.SerialException as e:
    print(f"Gagal membuka port: {e}")
