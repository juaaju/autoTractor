import csv
import time
import threading
import os
from datetime import datetime
from collections import deque
from gpsread import GPSReader  # Import class yang sudah ada
import pynmea2

class GPSDataLogger:
    def __init__(self, port: str, baudrate=9600, csv_filename=None, log_folder="gps_logs"):
        self.gps = GPSReader(port, baudrate)
        
        # Setup folder untuk log
        self.log_folder = log_folder
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
            print(f"Created log folder: {self.log_folder}")
        
        # Setup filename dengan path folder
        if csv_filename is None:
            filename = f"gps_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        else:
            filename = csv_filename
            
        self.csv_filename = os.path.join(self.log_folder, filename)
        
        # Untuk menghitung Hz/frequency
        self.timestamps = deque(maxlen=100)  # Simpan 100 timestamp terakhir
        self.valid_readings = 0
        self.total_readings = 0
        self.start_time = time.time()
        
        # CSV headers berdasarkan NMEA GPGGA/GNGGA fields
        self.csv_headers = [
            'timestamp_unix',      # Unix timestamp (1718185234.567)
            'timestamp_readable',  # Format: 2025-06-12 14:23:45.123
            'latitude',
            'longitude', 
            'gps_quality',
            'num_satellites',
            'horizontal_dilution',
            'altitude',
            'altitude_units',
            'geoid_height',
            'geoid_height_units',
            'dgps_time',
            'dgps_station_id',
            'raw_sentence',
            'hz_instantaneous',
            'hz_average'
        ]
        
        self.setup_csv()
        
    def setup_csv(self):
        """Setup CSV file dengan headers"""
        try:
            with open(self.csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(self.csv_headers)
            print(f"CSV file created: {self.csv_filename}")
        except Exception as e:
            print(f"Error creating CSV file: {e}")
    
    def calculate_hz(self):
        """Menghitung Hz instantaneous dan average"""
        current_time = time.time()
        
        # Hz instantaneous (berdasarkan 10 reading terakhir)
        hz_instantaneous = 0.0
        if len(self.timestamps) >= 2:
            recent_timestamps = list(self.timestamps)[-10:]  # 10 terakhir
            if len(recent_timestamps) >= 2:
                time_diff = recent_timestamps[-1] - recent_timestamps[0]
                if time_diff > 0:
                    hz_instantaneous = (len(recent_timestamps) - 1) / time_diff
        
        # Hz average (sejak mulai)
        elapsed_time = current_time - self.start_time
        hz_average = self.valid_readings / elapsed_time if elapsed_time > 0 else 0.0
        
        return hz_instantaneous, hz_average
    
    def parse_gps_data(self, raw_data):
        """Parse raw NMEA data untuk mendapatkan semua field GPGGA/GNGGA"""
        if not raw_data or not raw_data.startswith(("$GNGGA", "$GPGGA")):
            return None
            
        try:
            # Parse menggunakan pynmea2
            parsed = pynmea2.parse(raw_data.strip())
            
            # Extract semua field yang tersedia dari GPGGA/GNGGA
            gps_data = {
                'latitude': parsed.latitude,
                'longitude': parsed.longitude,
                'gps_quality': parsed.gps_qual,
                'num_satellites': parsed.num_sats,
                'horizontal_dilution': parsed.horizontal_dil,
                'altitude': parsed.altitude,
                'altitude_units': parsed.altitude_units,
                'geoid_height': parsed.geo_sep,
                'geoid_height_units': parsed.geo_sep_units,
                'dgps_time': getattr(parsed, 'age_gps_data', None),
                'dgps_station_id': getattr(parsed, 'ref_station_id', None),
                'raw_sentence': raw_data.strip()
            }
            
            return gps_data
            
        except Exception as e:
            print(f"Error parsing GPS data: {e}")
            return None
    
    def read_raw_gps(self):
        """Membaca raw GPS data langsung dari serial untuk parsing lengkap"""
        try:
            if self.gps.ser is None or not self.gps.ser.is_open:
                self.gps.connect()
                if self.gps.ser is None:
                    return None
                    
            raw_data = self.gps.ser.readline()
            if raw_data:
                return raw_data.decode('ascii', errors='ignore')
            return None
            
        except Exception as e:
            print(f"Error reading raw GPS data: {e}")
            return None
    
    def log_data(self, gps_data, hz_instantaneous, hz_average, gps_timestamp):
        """Log data ke CSV file"""
        try:
            with open(self.csv_filename, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                
                row = [
                    gps_timestamp,  # timestamp saat GPS data diterima (lebih akurat)
                    datetime.fromtimestamp(gps_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],  # format readable dengan ms
                    gps_data['latitude'],
                    gps_data['longitude'],
                    gps_data['gps_quality'],
                    gps_data['num_satellites'],
                    gps_data['horizontal_dilution'],
                    gps_data['altitude'],
                    gps_data['altitude_units'],
                    gps_data['geoid_height'],
                    gps_data['geoid_height_units'],
                    gps_data['dgps_time'],
                    gps_data['dgps_station_id'],
                    gps_data['raw_sentence'],
                    round(hz_instantaneous, 2),
                    round(hz_average, 2)
                ]
                
                writer.writerow(row)
                
        except Exception as e:
            print(f"Error writing to CSV: {e}")
    
    def print_status(self, gps_data, hz_instantaneous, hz_average):
        """Print status ke console"""
        print(f"\n--- GPS Data Update ---")
        print(f"Time: {datetime.now().strftime('%H:%M:%S')}")
        print(f"Lat: {gps_data['latitude']:.6f}, Lon: {gps_data['longitude']:.6f}")
        print(f"Quality: {gps_data['gps_quality']}, Satellites: {gps_data['num_satellites']}")
        print(f"Altitude: {gps_data['altitude']} {gps_data['altitude_units']}")
        print(f"HDOP: {gps_data['horizontal_dilution']}")
        print(f"Hz (instantaneous): {hz_instantaneous:.2f}")
        print(f"Hz (average): {hz_average:.2f}")
        print(f"Valid readings: {self.valid_readings}/{self.total_readings}")
    
    def run(self, print_console=True, log_every_update=True):
        """
        Main loop untuk membaca dan log GPS data
        Args:
            print_console: Print ke console atau tidak
            log_every_update: True = log setiap GPS update, False = log hanya koordinat yang berubah
        """
        print(f"Starting GPS data logging...")
        print(f"CSV file: {self.csv_filename}")
        print(f"Log mode: {'Every GPS update' if log_every_update else 'Only when coordinates change'}")
        print("Press Ctrl+C to stop")
        
        last_coordinates = (None, None)  # Track koordinat terakhir
        
        try:
            while True:
                # Baca raw GPS data
                raw_data = self.read_raw_gps()
                self.total_readings += 1
                
                if raw_data and raw_data.startswith(("$GNGGA", "$GPGGA")):
                    # Capture timestamp sesegera mungkin setelah data diterima
                    gps_timestamp = time.time()
                    
                    # Parse data
                    gps_data = self.parse_gps_data(raw_data)
                    
                    if gps_data and gps_data['latitude'] is not None and gps_data['longitude'] is not None:
                        current_coordinates = (gps_data['latitude'], gps_data['longitude'])
                        
                        # Tentukan apakah akan di-log berdasarkan mode
                        should_log = log_every_update or (current_coordinates != last_coordinates)
                        
                        if should_log:
                            # Update timestamps dan counters
                            self.timestamps.append(gps_timestamp)
                            self.valid_readings += 1
                            
                            # Hitung Hz
                            hz_instantaneous, hz_average = self.calculate_hz()
                            
                            # Log ke CSV
                            self.log_data(gps_data, hz_instantaneous, hz_average, gps_timestamp)
                            
                            # Print ke console jika diminta
                            if print_console:
                                self.print_status(gps_data, hz_instantaneous, hz_average)
                            
                            last_coordinates = current_coordinates
                
                # Minimal delay untuk mencegah CPU overload tapi tetap responsif
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n\nStopping GPS data logger...")
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.gps.close()
            print(f"Data saved to: {self.csv_filename}")
            print(f"Total valid readings: {self.valid_readings}/{self.total_readings}")

class GPSStatsMonitor:
    """Class terpisah untuk monitoring statistik real-time"""
    def __init__(self, logger):
        self.logger = logger
        self.running = False
        
    def start_monitoring(self):
        """Start monitoring thread untuk print statistik setiap 10 detik"""
        self.running = True
        monitor_thread = threading.Thread(target=self._monitor_loop)
        monitor_thread.daemon = True
        monitor_thread.start()
        
    def stop_monitoring(self):
        self.running = False
        
    def _monitor_loop(self):
        while self.running:
            time.sleep(10)  # Print stats setiap 10 detik
            if self.logger.valid_readings > 0:
                elapsed = time.time() - self.logger.start_time
                hz_avg = self.logger.valid_readings / elapsed
                print(f"\n=== GPS Stats (last 10s) ===")
                print(f"Runtime: {elapsed:.1f}s")
                print(f"Valid readings: {self.logger.valid_readings}/{self.logger.total_readings}")
                print(f"Success rate: {(self.logger.valid_readings/self.logger.total_readings)*100:.1f}%")
                print(f"Average Hz: {hz_avg:.2f}")

if __name__ == "__main__":
    # Konfigurasi
    GPS_PORT = "/dev/ttyUSB0"  # Sesuaikan dengan port GPS Anda
    BAUDRATE = 9600
    LOG_FOLDER = "gps_data_logs"  # Folder untuk menyimpan file CSV
    
    # Buat logger dengan custom folder
    logger = GPSDataLogger(GPS_PORT, BAUDRATE, log_folder=LOG_FOLDER)
    
    # Optional: Start stats monitor
    stats_monitor = GPSStatsMonitor(logger)
    stats_monitor.start_monitoring()
    
    try:
        # Mode logging:
        # log_every_update=True  -> Log setiap GPS update (default)
        # log_every_update=False -> Log hanya saat koordinat berubah (mengurangi lagging)
        logger.run(print_console=True, log_every_update=True)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stats_monitor.stop_monitoring()
