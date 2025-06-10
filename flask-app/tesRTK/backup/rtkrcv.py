#!/usr/bin/env python3
"""
Real-time GPS RTK Corrector menggunakan RTKLIB rtkrcv
rtkrcv adalah real-time RTK engine dari RTKLIB
"""

import time
import threading
import serial
import subprocess
import os
import tempfile
import socket
from io import BytesIO
from ntripclient import NtripClient
import pynmea2
import queue

class RTKRCVGPSCorrector:
    def __init__(self, gps_port='/dev/ttyUSB0', gps_baudrate=15200):
        self.rtcm_count = 0
        self.last_rtcm_time = 0
        
        # GPS data
        self.raw_gps_data = {}
        self.rtk_solution = {}
        self.gps_lock = threading.Lock()
        
        # GPS Serial connection
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self.gps_serial = None
        self.gps_running = False
        
        # RTKRCV configuration
        self.temp_dir = tempfile.mkdtemp()
        self.config_file = os.path.join(self.temp_dir, "rtkrcv.conf")
        self.solution_port = 9001  # Port untuk output RTKRCV
        self.rtcm_port = 9002      # Port untuk input RTCM
        
        # RTKRCV process
        self.rtkrcv_process = None
        self.rtkrcv_running = False
        
        # RTK Status
        self.rtk_status = "No Fix"
        
        # Create RTKRCV config
        self.create_rtkrcv_config()
        
        print(f"RTKRCV temp directory: {self.temp_dir}")
    
    def create_rtkrcv_config(self):
        """Buat file konfigurasi RTKRCV untuk real-time processing"""
        config_content = f"""# RTKRCV Configuration for Real-time RTK
console-passwd   =admin
console-timetype =gpst

# Input streams
inpstr1-type     =serial       # rover receiver
inpstr1-path     ={self.gps_port}:{self.gps_baudrate}:8:n:1
inpstr1-format   =ubx          # receiver format (ubx/novatel/oem4/oem3/hemis/stq/gw10/javad/nvs/binex/rt17/sbf/cmr)

inpstr2-type     =tcpsvr       # base station corrections  
inpstr2-path     =:{self.rtcm_port}
inpstr2-format   =rtcm3        # correction format

inpstr3-type     =off          # additional input stream
inpstr3-path     =
inpstr3-format   =sp3

# Output streams
outstr1-type     =tcpsvr       # solution output
outstr1-path     =:{self.solution_port}
outstr1-format   =llh          # solution format (llh/xyz/enu/nmea)

outstr2-type     =off          # additional output
outstr2-path     =
outstr2-format   =nmea

# Log streams  
logstr1-type     =off
logstr1-path     =
logstr2-type     =off  
logstr2-path     =
logstr3-type     =off
logstr3-path     =

# Processing options
pos1-posmode     =kinematic    # positioning mode (single/dgps/kinematic/static/movingbase/fixed/ppp-kine/ppp-static)
pos1-frequency   =l1+l2        # used frequencies (l1/l1+l2/l1+l2+l5)
pos1-soltype     =forward      # solution type (forward/backward/combined)
pos1-elmask      =15           # elevation mask angle (deg)
pos1-snrmask_r   =off          # SNR mask for rover
pos1-snrmask_b   =off          # SNR mask for base station
pos1-dynamics    =on           # dynamics model (on/off)
pos1-tidecorr    =off          # earth tide correction
pos1-ionoopt     =brdc         # ionosphere option (off/brdc/sbas/dual-freq/est-stec/ionex-tec/qzs-brdc)
pos1-tropopt     =saas         # troposphere option (off/saas/sbas/est-ztd/est-ztdgrad)
pos1-sateph      =brdc         # satellite ephemeris/clock (brdc/precise/brdc+sbas/brdc+ssrapc/brdc+ssrcom)

# Ambiguity resolution
pos2-armode      =continuous   # AR mode (off/continuous/instantaneous/fix-and-hold)
pos2-gloarmode   =on           # GLONASS AR mode (on/off/autocal)
pos2-bdsarmode   =on           # BeiDou AR mode (on/off)
pos2-arthres     =3            # AR validation threshold
pos2-arlockcnt   =0            # AR lock count threshold
pos2-arelmask    =0            # AR elevation mask angle (deg)
pos2-arminfix    =10           # AR min fix count
pos2-armaxiter   =1            # AR max iteration
pos2-elmaskhold  =0            # AR elevation mask angle during hold (deg)
pos2-aroutcnt    =5            # outage count to reset AR
pos2-maxage      =30           # max age of differential correction (s)
pos2-slipthres   =0.05         # slip threshold for cycle slip detection (m)
pos2-rejgdop     =30           # reject gdop threshold
pos2-niter       =1            # max number of iteration for point pos
pos2-baselen     =0            # baseline length constraint (0:off) (m)
pos2-basesig     =0            # baseline sigma (0:const) (m)

# Output options
out-solformat    =llh          # solution format (llh/xyz/enu/nmea)
out-outhead      =on           # output header (on/off)
out-outopt       =on           # output processing options (on/off)
out-timesys      =gpst         # time system (gpst/utc/jst)
out-timeform     =tow          # time format (tow/hms)
out-timendec     =3            # time decimal places
out-degform      =deg          # degree format (deg/dms)
out-height       =ellipsoidal  # height (ellipsoidal/geodetic)
out-geoid        =internal     # geoid model (internal/egm96/egm08_2.5/egm08_1/gsi2000)
out-solstatic    =all          # solution of static mode (all/single)
out-outstat      =off          # output solution status (off/state/residual)

# Statistics
stats-eratio1    =100          # code/phase error ratio
stats-eratio2    =100          # baseline error ratio  
stats-errphase   =0.003        # phase error factor (a) (m)
stats-errphaseel =0.003        # phase error factor (b) (m/sin(el))
stats-errphasebl =0            # phase error factor (c) (m/baseline)
stats-errdoppler =1            # doppler error factor (Hz)
stats-stdbias    =30           # initial bias uncertainty (m)
stats-stdiono    =0.03         # initial ionos uncertainty (m)
stats-stdtrop    =0.3          # initial tropo uncertainty (m)
stats-prnaccelh  =1            # process noise h accel (m/s/s)
stats-prnaccelv  =0.1          # process noise v accel (m/s/s)
stats-prnbias    =0.0001       # process noise bias (m/s)
stats-prniono    =0.001        # process noise iono (m/s)
stats-prntrop    =0.0001       # process noise tropo (m/s)
stats-prnpos     =0            # process noise pos (m/s)
stats-clkstab    =5e-12        # clock stability (s/s)

# Antenna positions (will be auto-detected from GPS)
ant1-postype     =llh          # rover antenna position type
ant1-pos1        =0            # rover antenna position-1 (deg/m)
ant1-pos2        =0            # rover antenna position-2 (deg/m)  
ant1-pos3        =0            # rover antenna position-3 (m)

ant2-postype     =llh          # base station antenna position type
ant2-pos1        =0            # base station antenna position-1 (deg/m)
ant2-pos2        =0            # base station antenna position-2 (deg/m)
ant2-pos3        =0            # base station antenna position-3 (m)

# Miscellaneous
misc-timeinterp  =on           # time interpolation (on/off)
misc-sbasatsel   =0            # SBAS satellite selection (0:all)
"""
        
        with open(self.config_file, 'w') as f:
            f.write(config_content)
    
    def start_rtkrcv(self):
        """Start RTKRCV real-time processing"""
        try:
            cmd = ['rtkrcv', '-o', self.config_file]
            
            self.rtkrcv_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            print("RTKRCV started successfully")
            self.rtkrcv_running = True
            
            # Start monitoring thread
            monitor_thread = threading.Thread(target=self.monitor_rtkrcv)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            return True
            
        except Exception as e:
            print(f"Error starting RTKRCV: {e}")
            return False
    
    def monitor_rtkrcv(self):
        """Monitor RTKRCV output untuk debugging"""
        while self.rtkrcv_running and self.rtkrcv_process:
            try:
                output = self.rtkrcv_process.stdout.readline()
                if output:
                    print(f"RTKRCV: {output.strip()}")
                else:
                    break
            except Exception as e:
                print(f"Error monitoring RTKRCV: {e}")
                break
    
    def connect_gps(self):
        """Koneksi ke GPS receiver - RTKRCV akan handle ini"""
        # RTKRCV akan langsung handle koneksi serial
        # Kita hanya perlu memverifikasi bahwa port tersedia
        try:
            test_serial = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=1
            )
            test_serial.close()
            print(f"GPS port {self.gps_port} is available")
            return True
        except Exception as e:
            print(f"GPS port {self.gps_port} error: {e}")
            return False
    
    def read_rtk_solution(self):
        """Thread untuk membaca solusi RTK dari RTKRCV via TCP"""
        try:
            # Connect ke RTKRCV solution output
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            
            # Tunggu RTKRCV ready
            time.sleep(2)
            
            sock.connect(('localhost', self.solution_port))
            print(f"Connected to RTKRCV solution stream on port {self.solution_port}")
            
            sock_file = sock.makefile('r')
            
            while self.rtkrcv_running:
                try:
                    line = sock_file.readline().strip()
                    if line:
                        self.parse_rtkrcv_solution(line)
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error reading RTK solution: {e}")
                    break
                    
        except Exception as e:
            print(f"Error connecting to RTKRCV solution: {e}")
        finally:
            try:
                sock.close()
            except:
                pass
    
    def parse_rtkrcv_solution(self, line):
        """Parse output solusi dari RTKRCV"""
        try:
            # RTKRCV LLH format: %Y/%m/%d %H:%M:%S.%f lat(deg) lon(deg) height(m) Q ns sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) age(s) ratio
            
            if line.startswith('%') or line.strip() == '':
                return
                
            parts = line.strip().split()
            if len(parts) >= 8:
                try:
                    # Parse date/time
                    date_str = parts[0] + ' ' + parts[1]
                    
                    # Parse coordinates
                    lat = float(parts[2])
                    lon = float(parts[3])
                    height = float(parts[4])
                    quality = int(parts[5])  # 1:Fix, 2:Float, 3:SBAS, 4:DGPS, 5:Single, 6:PPP
                    num_sats = int(parts[6])
                    
                    # Parse standard deviations jika tersedia
                    sdn = float(parts[7]) if len(parts) > 7 else 0.0
                    sde = float(parts[8]) if len(parts) > 8 else 0.0
                    sdu = float(parts[9]) if len(parts) > 9 else 0.0
                    age = float(parts[13]) if len(parts) > 13 else 0.0
                    ratio = float(parts[14]) if len(parts) > 14 else 0.0
                    
                    # Map quality flag ke status text
                    status_map = {
                        1: "RTK Fixed",
                        2: "RTK Float",
                        3: "SBAS",
                        4: "DGPS", 
                        5: "Single",
                        6: "PPP"
                    }
                    
                    with self.gps_lock:
                        self.rtk_solution = {
                            'latitude': lat,
                            'longitude': lon,
                            'altitude': height,
                            'quality': quality,
                            'status': status_map.get(quality, f"Unknown({quality})"),
                            'satellites': num_sats,
                            'std_north': sdn,
                            'std_east': sde,
                            'std_up': sdu,
                            'correction_age': age,
                            'ratio': ratio,
                            'timestamp': time.time(),
                            'rtcm_count': self.rtcm_count
                        }
                        
                        self.rtk_status = self.rtk_solution['status']
                        
                except (ValueError, IndexError) as e:
                    pass
                    
        except Exception as e:
            pass
    
    def send_rtcm_to_rtkrcv(self, rtcm_data):
        """Kirim data RTCM ke RTKRCV via TCP"""
        try:
            # Connect ke RTKRCV RTCM input port
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            sock.connect(('localhost', self.rtcm_port))
            sock.send(rtcm_data)
            sock.close()
            
        except Exception as e:
            # Connection error normal jika RTKRCV belum ready
            pass
    
    def on_rtcm_data(self, rtcm_data):
        """Callback ketika ada data RTCM baru"""
        self.rtcm_count += 1
        self.last_rtcm_time = time.time()
        
        # Kirim data RTCM ke RTKRCV
        if self.rtkrcv_running:
            self.send_rtcm_to_rtkrcv(rtcm_data)
    
    def read_raw_gps_for_comparison(self):
        """Baca GPS mentah untuk perbandingan (opsional)"""
        try:
            self.gps_serial = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=1
            )
            
            while self.gps_running:
                try:
                    if self.gps_serial.in_waiting > 0:
                        line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                        
                        if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                            try:
                                msg = pynmea2.parse(line)
                                if msg.latitude and msg.longitude:
                                    with self.gps_lock:
                                        self.raw_gps_data = {
                                            'latitude': float(msg.latitude),
                                            'longitude': float(msg.longitude),
                                            'altitude': float(msg.altitude) if msg.altitude else 0.0,
                                            'timestamp': time.time(),
                                            'quality': int(msg.gps_qual) if msg.gps_qual else 0,
                                            'satellites': int(msg.num_sats) if msg.num_sats else 0,
                                            'hdop': float(msg.horizontal_dil) if msg.horizontal_dil else 0.0
                                        }
                            except Exception:
                                pass
                                
                except Exception:
                    continue
                    
        except Exception as e:
            print(f"Raw GPS reading error: {e}")
        finally:
            if self.gps_serial:
                self.gps_serial.close()
    
    def print_gps_status(self):
        """Print status GPS dan RTK secara berkala"""
        while True:
            print("\n" + "="*70)
            print("RTKRCV REAL-TIME GPS RTK STATUS")
            print("="*70)
            
            with self.gps_lock:
                # Raw GPS Data (opsional, untuk perbandingan)
                if self.raw_gps_data:
                    print("Raw GPS Data (for comparison):")
                    print(f"  Position: {self.raw_gps_data['latitude']:.6f}, {self.raw_gps_data['longitude']:.6f}")
                    print(f"  Altitude: {self.raw_gps_data['altitude']:.2f}m")
                    print(f"  Quality: {self.raw_gps_data.get('quality', 0)} | Satellites: {self.raw_gps_data.get('satellites', 0)}")
                
                # RTKRCV RTK Solution
                if self.rtk_solution:
                    print(f"RTKRCV RTK Solution:")
                    print(f"  Position: {self.rtk_solution['latitude']:.6f}, {self.rtk_solution['longitude']:.6f}")
                    print(f"  Altitude: {self.rtk_solution['altitude']:.2f}m")
                    print(f"  RTK Status: {self.rtk_solution['status']}")
                    print(f"  Quality: {self.rtk_solution['quality']} | Satellites: {self.rtk_solution['satellites']}")
                    print(f"  Accuracy (1σ): N={self.rtk_solution['std_north']:.3f}m, E={self.rtk_solution['std_east']:.3f}m, U={self.rtk_solution['std_up']:.3f}m")
                    print(f"  Correction Age: {self.rtk_solution['correction_age']:.1f}s")
                    print(f"  AR Ratio: {self.rtk_solution['ratio']:.1f}")
                    
                    # Calculate difference jika ada raw GPS
                    if self.raw_gps_data:
                        lat_diff = abs(self.rtk_solution['latitude'] - self.raw_gps_data['latitude'])
                        lon_diff = abs(self.rtk_solution['longitude'] - self.raw_gps_data['longitude'])
                        alt_diff = abs(self.rtk_solution['altitude'] - self.raw_gps_data['altitude'])
                        print(f"  Improvement over raw GPS:")
                        print(f"    Latitude: {lat_diff*111000:.3f}m")
                        print(f"    Longitude: {lon_diff*111000:.3f}m")
                        print(f"    Altitude: {alt_diff:.3f}m")
                else:
                    print(f"RTKRCV Status: Waiting for solution...")
                
                print(f"\nRTCM Info:")
                print(f"  Packets received: {self.rtcm_count}")
                print(f"  Last RTCM: {time.time() - self.last_rtcm_time:.1f}s ago" if self.last_rtcm_time > 0 else "  Last RTCM: Never")
                print(f"  RTKRCV Running: {self.rtkrcv_running}")
            
            time.sleep(2)  # Update setiap 5 detik
    
    def stop(self):
        """Stop semua thread dan RTKRCV"""
        self.gps_running = False
        self.rtkrcv_running = False
        
        if self.rtkrcv_process:
            try:
                self.rtkrcv_process.terminate()
                self.rtkrcv_process.wait(timeout=5)
            except:
                self.rtkrcv_process.kill()

def main():
    print("=== RTKRCV Real-time GPS RTK Corrector ===")
    print("Menggunakan RTKLIB rtkrcv untuk real-time RTK processing")
    
    # Cek apakah rtkrcv tersedia
    try:
        result = subprocess.run(['rtkrcv', '-h'], capture_output=True, timeout=5)
        print("✓ RTKRCV detected")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("❌ RTKRCV not found!")
        print("Install RTKLIB with: sudo apt install rtklib")
        return
    
    # Inisialisasi corrector
    corrector = RTKRCVGPSCorrector(gps_port='/dev/ttyUSB0', gps_baudrate=15200)
    
    # Cek GPS port
    if not corrector.connect_gps():
        print("❌ Cannot access GPS port")
        return
    
    # Start RTKRCV
    if not corrector.start_rtkrcv():
        print("❌ Failed to start RTKRCV")
        return
    
    # Konfigurasi NTRIP
    ntrip_config = {
        'user': 'toor:toor123456',  # Ganti dengan kredensial yang benar
        'caster': 'nrtk.big.go.id',
        'port': 2001,
        'mountpoint': '/vrs-rtcm3',
        'lat': -7.2839114,
        'lon': 112.7961259,
        'height': 10.0,
        'verbose': False,
        'out': BytesIO()
    }
    
    # Modifikasi NtripClient untuk callback
    def new_receive_data(self):
        lastGGATime = time.time()
        
        try:
            while True:
                current_time = time.time()
                
                # Update posisi GGA dari RTK solution jika tersedia
                if corrector.rtk_solution:
                    with corrector.gps_lock:
                        self.setPosition(
                            corrector.rtk_solution['latitude'],
                            corrector.rtk_solution['longitude']
                        )
                        self.height = corrector.rtk_solution['altitude']
                
                if current_time - lastGGATime > 10:
                    self.socket.send(self.getGGAString().encode())
                    lastGGATime = current_time
                
                self.socket.settimeout(1.0)
                try:
                    data = self.socket.recv(self.buffer)
                    if not data:
                        break
                    
                    corrector.on_rtcm_data(data)
                    
                except Exception:
                    continue
                    
        except KeyboardInterrupt:
            print("NTRIP stopped by user")
        finally:
            if self.socket:
                self.socket.close()
    
    # Replace method
    NtripClient.receiveData = new_receive_data
    
    # Start threads
    # RTK solution reader
    rtk_thread = threading.Thread(target=corrector.read_rtk_solution)
    rtk_thread.daemon = True
    rtk_thread.start()
    
    # Raw GPS reader (opsional, untuk perbandingan)
    corrector.gps_running = True
    # gps_thread = threading.Thread(target=corrector.read_raw_gps_for_comparison)
    # gps_thread.daemon = True
    # gps_thread.start()
    
    # Status monitor
    status_thread = threading.Thread(target=corrector.print_gps_status)
    status_thread.daemon = True
    status_thread.start()
    
    # NTRIP client
    client = NtripClient(**ntrip_config)
    ntrip_thread = threading.Thread(target=client.readLoop)
    ntrip_thread.daemon = True
    ntrip_thread.start()
    
    print(f"System started:")
    print(f"- GPS: {corrector.gps_port}")
    print(f"- NTRIP: {ntrip_config['caster']}")
    print(f"- RTKRCV Config: {corrector.config_file}")
    print(f"- Solution Port: {corrector.solution_port}")
    print(f"- RTCM Port: {corrector.rtcm_port}")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping system...")
        corrector.stop()

if __name__ == '__main__':
    main()