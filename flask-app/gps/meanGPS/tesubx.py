from serial import Serial
from pyubx2 import UBXReader
import time
from datetime import datetime

# Membuka koneksi serial
stream = Serial('/dev/ttyUSB0', 9600)
ubr = UBXReader(stream)

# Dictionary untuk menyimpan data terbaru
latest_data = {
    'Timestamp': None,
    'Latitude': None,
    'Longitude': None,
    'NumSV': None,
    'FixType': None,
    'GDOP': None,
    'PDOP': None,
    'HDOP': None,
    'VDOP': None
}

try:
    print("Mulai membaca data GPS... (Tekan Ctrl+C untuk berhenti)")
    print("=" * 70)
    print("| Timestamp            | Lat        | Lon        | Fix | Sats | PDOP  | HDOP  |")
    print("|" + "-" * 68 + "|")
    
    while True:
        # Membaca data dari receiver
        (raw_data, parsed_data) = ubr.read()
        
        # Update timestamp saat ini
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        latest_data['Timestamp'] = current_time
        
        # Memeriksa apakah pesan adalah tipe NAV-PVT
        if parsed_data.identity == "NAV-PVT":
            latest_data['Latitude'] = parsed_data.lat
            latest_data['Longitude'] = parsed_data.lon
            latest_data['NumSV'] = parsed_data.numSV
            latest_data['FixType'] = parsed_data.fixType
            
            # Print info PVT tanpa DOP jika belum ada data DOP
            if latest_data['PDOP'] is None:
                print(f"| {latest_data['Timestamp']} | {latest_data['Latitude']:10.6f} | {latest_data['Longitude']:10.6f} | {latest_data['FixType']:3d} | {latest_data['NumSV']:4d} |  ?.?? |  ?.?? |")
            else:
                # Print info lengkap jika sudah ada data DOP
                print(f"| {latest_data['Timestamp']} | {latest_data['Latitude']:10.6f} | {latest_data['Longitude']:10.6f} | {latest_data['FixType']:3d} | {latest_data['NumSV']:4d} | {latest_data['PDOP']:5.2f} | {latest_data['HDOP']:5.2f} |")
            
        # Jika pesan adalah tipe NAV-DOP
        elif parsed_data.identity == "NAV-DOP":
            latest_data['GDOP'] = parsed_data.gDOP
            latest_data['PDOP'] = parsed_data.pDOP
            latest_data['HDOP'] = parsed_data.hDOP
            latest_data['VDOP'] = parsed_data.vDOP
            
            # Print update DOP saja jika sudah ada data posisi
            if latest_data['Latitude'] is not None:
                print(f"| {latest_data['Timestamp']} | {latest_data['Latitude']:10.6f} | {latest_data['Longitude']:10.6f} | {latest_data['FixType']:3d} | {latest_data['NumSV']:4d} | {latest_data['PDOP']:5.2f} | {latest_data['HDOP']:5.2f} |")
        
        # Jeda kecil untuk mengurangi penggunaan CPU
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\nProgram dihentikan.")
finally:
    # Menutup koneksi serial saat selesai
    stream.close()