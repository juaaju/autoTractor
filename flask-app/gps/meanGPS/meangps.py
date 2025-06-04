from gpsread import GPSReader
import time
import csv
from math import radians, cos
import threading
from queue import Queue
import copy
import pymap3d as pm

# Konfigurasi
GPS_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]
LAT_REF = -7.283640
LNG_REF = 112.796467
GPS_DATA_FILE = 'data_gps.csv'
MEAN_GPS_FILE = 'data_meangps.csv'
READ_INTERVAL = 0.1  # Interval pembacaan dalam detik

def simple_latlon2xy(lat_ref, lon_ref, lat, lon):
    """
    Menghitung koordinat x, y dalam meter relatif terhadap titik referensi (lat_ref, lon_ref)
    """
    R = 6371000  # jari-jari bumi (meter)
    dlat = radians(lat - lat_ref)
    dlon = radians(lon - lon_ref)
    lat_avg = radians((lat + lat_ref) / 2)
    x = R * dlon * cos(lat_avg)
    y = R * dlat
    return x, y

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    e, n, u = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
    return e, n  # east (x), north (y)

def initialize_csv_files():
    """Inisialisasi file CSV dengan header."""
    with open(GPS_DATA_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'GPS_ID', 'Latitude', 'Longitude', 'X', 'Y', 'Fix', 'HDOP'])
    
    with open(MEAN_GPS_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'Mean_X', 'Mean_Y', 'Num_GPS_Used'])

class GPSReaderThread(threading.Thread):
    """Thread untuk membaca data GPS secara paralel."""
    
    def __init__(self, gps_id, gps_reader, data_queue):
        threading.Thread.__init__(self)
        self.gps_id = gps_id
        self.gps_reader = gps_reader
        self.data_queue = data_queue
        self.daemon = True  # Thread akan otomatis berhenti ketika program utama berhenti
        self.stop_event = threading.Event()
    
    def run(self):
        while not self.stop_event.is_set():
            try:
                coords = self.gps_reader.read()
                if coords:
                    lat, lng = coords
                    x, y = latlon_to_xy(LAT_REF, LNG_REF,0, lat, lng,0)
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    
                    # Taruh data di queue untuk diproses oleh thread utama
                    self.data_queue.put({
                        'gps_id': self.gps_id,
                        'timestamp': timestamp,
                        'lat': lat,
                        'lng': lng,
                        'x': x,
                        'y': y
                    })
            except Exception as e:
                print(f"Error reading from GPS {self.gps_id}: {str(e)}")
            
            # Interval kecil antara pembacaan
            time.sleep(READ_INTERVAL)
    
    def stop(self):
        self.stop_event.set()

def main():
    # Inisialisasi GPS readers
    gps_readers = []
    gps_threads = []
    data_queue = Queue()
    
    try:
        # Inisialisasi semua GPS readers
        for i, port in enumerate(GPS_PORTS):
            try:
                reader = GPSReader(port=port)
                gps_readers.append(reader)
                
                # Buat dan mulai thread untuk setiap GPS reader
                thread = GPSReaderThread(i+1, reader, data_queue)
                thread.start()
                gps_threads.append(thread)
                
                print(f"GPS Reader {i+1} initialized on port {port}")
            except Exception as e:
                print(f"Failed to initialize GPS on port {port}: {str(e)}")
        
        if not gps_readers:
            print("No GPS readers available. Exiting.")
            return
            
        # Inisialisasi file CSV
        initialize_csv_files()
        
        # Buka file untuk menulis data
        with open(GPS_DATA_FILE, mode='a', newline='') as gps_file, \
             open(MEAN_GPS_FILE, mode='a', newline='') as mean_file:
            
            gps_writer = csv.writer(gps_file)
            mean_writer = csv.writer(mean_file)
            
            print("GPS monitoring started. Press Ctrl+C to stop.")
            
            # Untuk mengelompokkan data berdasarkan interval waktu
            current_interval = ""
            interval_data = {}
            
            while True:
                # Ambil data dari queue (non-blocking)
                try:
                    while not data_queue.empty():
                        data = data_queue.get(block=False)
                        
                        # Tulis ke file GPS data
                        gps_writer.writerow([
                            data['timestamp'], 
                            data['gps_id'], 
                            data['lat'], 
                            data['lng'], 
                            data['x'], 
                            data['y']
                        ])
                        gps_file.flush()  # Pastikan data langsung ditulis ke disk
                        
                        print(f"GPS {data['gps_id']}: {data['lat']}, {data['lng']}")
                        
                        # Grup berdasarkan interval waktu (misalnya per detik)
                        interval = data['timestamp']  # Atau bisa dimodifikasi untuk interval lain
                        
                        if interval != current_interval:
                            # Interval baru, proses interval sebelumnya jika ada data
                            if interval_data and current_interval:
                                process_interval_data(interval_data, current_interval, mean_writer, mean_file)
                                
                            # Reset untuk interval baru
                            current_interval = interval
                            interval_data = {}
                        
                        # Simpan data untuk interval ini
                        interval_data[data['gps_id']] = data
                        
                except Exception as e:
                    print(f"Error processing data: {str(e)}")
                
                time.sleep(0.1)  # Sedikit jeda agar tidak membebani CPU
                
    except KeyboardInterrupt:
        print("\nStopping GPS monitoring...")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        # Hentikan semua thread
        for thread in gps_threads:
            thread.stop()
            
        # Tunggu semua thread berhenti (opsional, karena thread adalah daemon)
        for thread in gps_threads:
            thread.join(timeout=1.0)
            
        # Tutup semua GPS readers
        for gps in gps_readers:
            try:
                gps.close()
            except:
                pass
                
def process_interval_data(interval_data, timestamp, mean_writer, mean_file):
    """Proses data dari satu interval waktu untuk menghitung rata-rata posisi."""
    if not interval_data:
        return
        
    sum_x = sum(data['x'] for data in interval_data.values())
    sum_y = sum(data['y'] for data in interval_data.values())
    count = len(interval_data)
    
    mean_x = sum_x / count
    mean_y = sum_y / count
    
    # Tulis ke file rata-rata GPS
    mean_writer.writerow([timestamp, mean_x, mean_y, count])
    mean_file.flush()  # Pastikan data langsung ditulis ke disk
    
    print(f"Mean position at {timestamp}: X={mean_x:.2f}, Y={mean_y:.2f} (using {count} GPS)")

if __name__ == "__main__":
    main()
