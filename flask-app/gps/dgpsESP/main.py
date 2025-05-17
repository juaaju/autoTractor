import network
import socket
import machine
import time

# Konfigurasi GPS
gps_serial = machine.UART(2, baudrate=9600, tx=17, rx=16)

# 1. Buat access point
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="ESP32_GPS_HOTSPOT", password="12345678", authmode=network.AUTH_WPA2_PSK)
print("Access Point aktif. IP:", ap.ifconfig()[0])

# 2. Buat web server
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print('Server GPS API aktif di', addr)

# Buffer untuk menyimpan data NMEA terbaru
nmea_buffer = []
max_buffer_size = 8  # Menyimpan 10 kalimat NMEA terakhir

# Main loop
while True:
    # Baca data GPS jika tersedia
    if gps_serial.any():
        line = gps_serial.readline()
        try:
            nmea_sentence = line.decode('utf-8').strip()
            # Hanya simpan jika valid dan dimulai dengan $
            if nmea_sentence and nmea_sentence.startswith('$'):
                # Tambahkan ke buffer, hapus yang lama jika penuh
                nmea_buffer.append(nmea_sentence)
                if len(nmea_buffer) > max_buffer_size:
                    nmea_buffer.pop(0)
        except Exception as e:
            print("Error membaca GPS:", e)
    
    # Periksa koneksi client
    try:
        # Set non-blocking timeout
        s.settimeout(0.01)
        try:
            cl, addr = s.accept()
            print('Client connected from', addr)
            request = cl.recv(1024)
            
            # Buat JSON dari data NMEA
            nmea_json = '{"nmea_data":['
            for i, sentence in enumerate(nmea_buffer):
                nmea_json += f'"{sentence}"'
                if i < len(nmea_buffer) - 1:
                    nmea_json += ','
            nmea_json += '],"timestamp":"' + str(time.time()) + '"}'
            
            # Kirim respons
            http_response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\n\r\n" + nmea_json
            cl.send(http_response)
            cl.close()
        except OSError:
            # Timeout pada accept - normal
            pass
            
    except Exception as e:
        print("Server error:", e)
    
    # Delay minimal
    time.sleep(0.01)