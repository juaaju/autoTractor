#!/usr/bin/env python3
"""
GPS RTK Corrector menggunakan RTKLIB untuk koreksi sesungguhnya
"""

import time
import threading
import serial
import subprocess
import os
import tempfile
from io import BytesIO
from ntripclient import NtripClient
import pynmea2
import queue

class RTKLIBGPSCorrector:
    def __init__(self, gps_port='/dev/ttyUSB0', gps_baudrate=115200):
        self.rtcm_count = 0
        self.last_rtcm_time = 0
        
        # GPS data
        self.raw_gps_data = {}
        self.corrected_gps_data = {}
        self.gps_lock = threading.Lock()
        
        # GPS Serial connection
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self.gps_serial = None
        self.gps_running = False
        
        # RTKLIB files
        self.temp_dir = tempfile.mkdtemp()
        self.rover_obs_file = os.path.join(self.temp_dir, "rover.obs")
        self.rtcm_file = os.path.join(self.temp_dir, "corrections.rtcm3")
        self.solution_file = os.path.join(self.temp_dir, "solution.pos")
        self.config_file = os.path.join(self.temp_dir, "rtkpost.conf")
        
        # RTK Status
        self.rtk_solution = {}
        self.rtk_status = "No Fix"
        
        # Create RTKLIB config
        self.create_rtklib_config()
        
        print(f"RTKLIB temp directory: {self.temp_dir}")
    
    def create_rtklib_config(self):
        """Buat file konfigurasi RTKLIB"""
        config_content = """# RTKLIB Configuration for Real-time RTK
pos1-posmode     =kinematic    # positioning mode (0:single,1:dgps,2:kinematic,3:static,4:movingbase,5:fixed,6:ppp-kine,7:ppp-static)
pos1-frequency   =l1+l2        # used frequencies (1:l1,2:l1+l2,3:l1+l2+l5)
pos1-soltype     =forward      # solution type (0:forward,1:backward,2:combined)
pos1-elmask      =15           # elevation mask angle (deg)
pos1-snrmask_r   =off          # SNR mask for rover (0:off,1:on)
pos1-snrmask_b   =off          # SNR mask for base station (0:off,1:on)
pos1-snrmask_L1  =0,0,0,0,0,0,0,0,0
pos1-snrmask_L2  =0,0,0,0,0,0,0,0,0
pos1-snrmask_L5  =0,0,0,0,0,0,0,0,0
pos1-dynamics    =on           # dynamics model (0:none,1:velociy,2:accel)
pos1-tidecorr    =off          # earth tide correction (0:off,1:solid,2:solid+otl+pole)
pos1-ionoopt     =brdc         # ionosphere option (0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc)
pos1-tropopt     =saas         # troposphere option (0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad)
pos1-sateph      =brdc         # satellite ephemeris/clock (0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom)
pos1-posopt1     =off          # satellite PCV (0:off,1:on)
pos1-posopt2     =off          # receiver antenna PCV (0:off,1:on)
pos1-posopt3     =off          # phase windup correction (0:off,1:on)
pos1-posopt4     =off          # reject eclipse satellite (0:off,1:on)
pos1-posopt5     =off          # RAIM FDE (0:off,1:on)
pos1-exclsats    =              # excluded satellites
pos1-navsys      =1            # navigation system(s) (1:gps+2:sbas+4:glo+8:gal+16:qzs+32:comp)

pos2-armode      =continuous   # AR mode (0:off,1:continuous,2:instantaneous,3:fix-and-hold)
pos2-gloarmode   =on           # GLONASS AR mode (0:off,1:on,2:autocal)
pos2-bdsarmode   =on           # BeiDou AR mode (0:off,1:on)
pos2-arfilter    =on           # AR filtering (0:off,1:on)
pos2-arthres     =3            # AR validation threshold
pos2-arlockcnt   =0            # AR lock count threshold
pos2-arelmask    =0            # AR elevation mask angle (deg)
pos2-arminfix    =10           # AR min fix count
pos2-armaxiter   =1            # AR max iteration
pos2-elmaskhold  =0            # AR elevation mask angle during hold (deg)
pos2-aroutcnt    =5            # outage count to reset AR
pos2-maxage      =30           # max age of differential correction (s)
pos2-syncsol     =off          # solution sync mode (0:off,1:on)
pos2-slipthres   =0.05         # slip threshold for cycle slip detection (m)
pos2-rejionno    =500          # reject ionospheric correction threshold (m)
pos2-rejgdop     =30           # reject gdop threshold
pos2-niter       =1            # max number of iteration for point pos
pos2-baselen     =0            # baseline length constraint {0:off} (m)
pos2-basesig     =0            # baseline sigma {0:const} (m)

out-solformat    =llh          # solution format (0:llh,1:xyz,2:enu,3:nmea)
out-outhead      =on           # output header (0:off,1:on)
out-outopt       =on           # output processing options (0:off,1:on)
out-timesys      =gpst         # time system (0:gpst,1:utc,2:jst)
out-timeform     =tow          # time format (0:tow,1:hms)
out-timendec     =3            # time decimal places
out-degform      =deg          # degree format (0:deg,1:dms)
out-fieldsep     =             # field separator
out-height       =ellipsoidal  # height (0:ellipsoidal,1:geodetic)
out-geoid        =internal     # geoid model (0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000)
out-solstatic    =all          # solution of static mode (0:all,1:single)
out-nmeaintv1    =0            # NMEA RMC,GGA output interval (s)
out-nmeaintv2    =0            # NMEA GSV output interval (s)
out-outstat      =off          # output solution status (0:off,1:state,2:residual)

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

ant1-postype     =llh          # rover antenna position type (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw)
ant1-pos1        =0            # rover antenna position-1 (deg|m)
ant1-pos2        =0            # rover antenna position-2 (deg|m)
ant1-pos3        =0            # rover antenna position-3 (m|m)
ant1-anttype     =*            # rover antenna type
ant1-antdelu     =0            # rover antenna delta-u (m)
ant1-antdeln     =0            # rover antenna delta-n (m)
ant1-antdele     =0            # rover antenna delta-e (m)

ant2-postype     =llh          # base station antenna position type (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw)
ant2-pos1        =0            # base station antenna position-1 (deg|m)
ant2-pos2        =0            # base station antenna position-2 (deg|m)
ant2-pos3        =0            # base station antenna position-3 (m|m)
ant2-anttype     =*            # base station antenna type
ant2-antdelu     =0            # base station antenna delta-u (m)
ant2-antdeln     =0            # base station antenna delta-n (m)
ant2-antdele     =0            # base station antenna delta-e (m)

misc-timeinterp  =on           # time interpolation (0:off,1:on)
misc-sbasatsel   =0            # SBAS satellite selection (0:all)
misc-rnxopt1     =             # RINEX options (1) rover
misc-rnxopt2     =             # RINEX options (2) base station
misc-pppopt      =             # PPP options
"""
        
        with open(self.config_file, 'w') as f:
            f.write(config_content)
    
    def connect_gps(self):
        """Koneksi ke GPS receiver via USB"""
        try:
            self.gps_serial = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            print(f"GPS connected to {self.gps_port}")
            return True
        except Exception as e:
            print(f"Error connecting to GPS: {e}")
            return False
    
    def read_gps_data(self):
        """Thread untuk membaca data GPS dan menyimpan ke file RINEX/OBS"""
        if not self.connect_gps():
            return
            
        self.gps_running = True
        print("Started reading GPS data...")
        
        # Buka file untuk menyimpan raw GPS data
        gps_raw_file = open(os.path.join(self.temp_dir, "gps_raw.log"), 'w')
        
        while self.gps_running:
            try:
                if self.gps_serial.in_waiting > 0:
                    line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                    
                    # Log semua data GPS
                    gps_raw_file.write(line + '\n')
                    gps_raw_file.flush()
                    
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
                                    
                                # Trigger RTK processing jika ada RTCM data
                                if self.rtcm_count > 0:
                                    self.process_rtk()
                                    
                        except Exception as e:
                            pass
                            
            except Exception as e:
                print(f"Error reading GPS: {e}")
                time.sleep(1)
                
        gps_raw_file.close()
        if self.gps_serial:
            self.gps_serial.close()
    
    def on_rtcm_data(self, rtcm_data):
        """Callback ketika ada data RTCM baru"""
        self.rtcm_count += 1
        self.last_rtcm_time = time.time()
        
        # Simpan data RTCM ke file
        with open(self.rtcm_file, 'ab') as f:
            f.write(rtcm_data)
        
        # Kirim ke GPS receiver jika mendukung RTK
        if self.gps_serial and self.gps_serial.is_open:
            try:
                self.gps_serial.write(rtcm_data)
            except Exception as e:
                print(f"Error sending RTCM to GPS: {e}")
    
    def convert_nmea_to_obs(self):
        """Convert NMEA log ke RINEX OBS menggunakan RTKLIB"""
        try:
            gps_raw_file = os.path.join(self.temp_dir, "gps_raw.log")
            if not os.path.exists(gps_raw_file):
                return False
                
            # Gunakan convbin dari RTKLIB untuk convert NMEA ke RINEX
            cmd = [
                'convbin', 
                '-r', 'nmea',
                '-o', self.rover_obs_file,
                gps_raw_file
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            return result.returncode == 0
            
        except Exception as e:
            print(f"Error converting NMEA to OBS: {e}")
            return False
    
    def process_rtk(self):
        """Proses RTK menggunakan RTKLIB rnx2rtkp"""
        try:
            # Convert NMEA ke RINEX OBS
            if not self.convert_nmea_to_obs():
                return
                
            # Cek apakah file yang diperlukan ada
            if not os.path.exists(self.rover_obs_file) or not os.path.exists(self.rtcm_file):
                return
                
            # Jalankan rnx2rtkp untuk pemrosesan RTK
            # rnx2rtkp format: rnx2rtkp [options] rover_obs base_obs nav_file
            cmd = [
                'rnx2rtkp',
                '-k', self.config_file,  # config file
                '-o', self.solution_file, # output file
                '-p', '0',               # positioning mode (0:single, 1:dgps, 2:kinematic, 3:static)
                '-m', '15',              # elevation mask (degrees)
                '-f', '2',               # frequency (1:L1, 2:L1+L2)
                '-n',                    # output solution status
                self.rover_obs_file,     # rover observation file
                self.rtcm_file          # base station data (RTCM)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.parse_rtk_solution()
            else:
                print(f"rnx2rtkp error: {result.stderr}")
                # Print more debug info
                if result.stdout:
                    print(f"stdout: {result.stdout}")
                
        except Exception as e:
            print(f"Error processing RTK: {e}")
    
    def parse_rtk_solution(self):
        """Parse hasil solusi RTK dari file output RTKPOST"""
        try:
            if not os.path.exists(self.solution_file):
                return
                
            with open(self.solution_file, 'r') as f:
                lines = f.readlines()
                
            # Ambil solusi terakhir
            for line in reversed(lines):
                if line.startswith('%') or line.strip() == '':
                    continue
                    
                parts = line.strip().split()
                if len(parts) >= 7:
                    try:
                        # Format: GPST week seconds lat(deg) lon(deg) height(m) Q ns sdn(m) sde(m) sdu(m)
                        week = parts[0]
                        seconds = float(parts[1])
                        lat = float(parts[2])
                        lon = float(parts[3])
                        height = float(parts[4])
                        quality = int(parts[5])  # 1:Fix, 2:Float, 3:SBAS, 4:DGPS, 5:Single, 6:PPP
                        num_sats = int(parts[6])
                        
                        # Standard deviations
                        sdn = float(parts[7]) if len(parts) > 7 else 0
                        sde = float(parts[8]) if len(parts) > 8 else 0
                        sdu = float(parts[9]) if len(parts) > 9 else 0
                        
                        # Tentukan status RTK berdasarkan quality flag
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
                                'timestamp': time.time(),
                                'rtcm_count': self.rtcm_count
                            }
                            
                            self.rtk_status = self.rtk_solution['status']
                        
                        break
                        
                    except (ValueError, IndexError) as e:
                        continue
                        
        except Exception as e:
            print(f"Error parsing RTK solution: {e}")
    
    def print_gps_status(self):
        """Print status GPS dan RTK secara berkala"""
        while True:
            print("\n" + "="*70)
            print("RTKLIB REAL-TIME GPS RTK STATUS")
            print("="*70)
            
            with self.gps_lock:
                # Raw GPS Data
                if self.raw_gps_data:
                    print("Raw GPS Data:")
                    print(f"  Position: {self.raw_gps_data['latitude']:.6f}, {self.raw_gps_data['longitude']:.6f}")
                    print(f"  Altitude: {self.raw_gps_data['altitude']:.2f}m")
                    print(f"  Quality: {self.raw_gps_data.get('quality', 0)} | Satellites: {self.raw_gps_data.get('satellites', 0)}")
                    print(f"  HDOP: {self.raw_gps_data.get('hdop', 0.0):.2f}")
                else:
                    print("Raw GPS Data: No data received")
                
                # RTK Solution from RTKLIB
                if self.rtk_solution:
                    print(f"\nRTKLIB Solution:")
                    print(f"  Position: {self.rtk_solution['latitude']:.6f}, {self.rtk_solution['longitude']:.6f}")
                    print(f"  Altitude: {self.rtk_solution['altitude']:.2f}m")
                    print(f"  RTK Status: {self.rtk_solution['status']}")
                    print(f"  Quality: {self.rtk_solution['quality']} | Satellites: {self.rtk_solution['satellites']}")
                    print(f"  Accuracy (1σ): N={self.rtk_solution['std_north']:.3f}m, E={self.rtk_solution['std_east']:.3f}m, U={self.rtk_solution['std_up']:.3f}m")
                    
                    # Hitung perbedaan dengan raw GPS
                    if self.raw_gps_data:
                        lat_diff = abs(self.rtk_solution['latitude'] - self.raw_gps_data['latitude'])
                        lon_diff = abs(self.rtk_solution['longitude'] - self.raw_gps_data['longitude'])
                        alt_diff = abs(self.rtk_solution['altitude'] - self.raw_gps_data['altitude'])
                        print(f"  Correction Applied:")
                        print(f"    Latitude: {lat_diff*111000:.3f}m")
                        print(f"    Longitude: {lon_diff*111000:.3f}m")
                        print(f"    Altitude: {alt_diff:.3f}m")
                else:
                    print(f"\nRTKLIB Status: Processing...")
                
                print(f"\nRTCM Info:")
                print(f"  Packets received: {self.rtcm_count}")
                print(f"  Last RTCM: {time.time() - self.last_rtcm_time:.1f}s ago" if self.last_rtcm_time > 0 else "  Last RTCM: Never")
                print(f"  Files: {self.temp_dir}")
            
            time.sleep(2)  # Update setiap 5 detik
    
    def stop(self):
        """Stop semua thread dan cleanup"""
        self.gps_running = False
        # Cleanup temp files jika diperlukan
        # shutil.rmtree(self.temp_dir)

def main():
    print("=== RTKLIB Real GPS RTK Corrector ===")
    print("Menggunakan RTKLIB untuk koreksi RTK yang sesungguhnya")
    
    # Cek instalasi RTKLIB
    rtklib_binary = None
    available_binaries = ['rnx2rtkp', 'rtkrcv', 'convbin', 'str2str']
    
    for binary in available_binaries:
        try:
            result = subprocess.run([binary, '-h'], capture_output=True, timeout=5)
            print(f"✓ Found RTKLIB binary: {binary}")
            if binary == 'rnx2rtkp':  # Prioritas untuk rnx2rtkp
                rtklib_binary = binary
                break
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
    
    if not rtklib_binary:
        print("❌ RTKLIB rnx2rtkp not found!")
        print("Available RTKLIB tools found:")
        for binary in available_binaries:
            try:
                subprocess.run([binary, '-h'], capture_output=True, timeout=2)
                print(f"  ✓ {binary}")
            except:
                print(f"  ❌ {binary}")
        return
    
    print(f"Using RTKLIB: {rtklib_binary}")
    print("Note: rnx2rtkp adalah post-processing tool, untuk real-time gunakan simple_rtk_corrector.py")
    
    # Inisialisasi corrector
    corrector = RTKLIBGPSCorrector(gps_port='/dev/ttyUSB0', gps_baudrate=115200)
    
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
                
                # Update posisi GGA dari GPS asli
                if corrector.raw_gps_data:
                    with corrector.gps_lock:
                        self.setPosition(
                            corrector.raw_gps_data['latitude'],
                            corrector.raw_gps_data['longitude']
                        )
                        self.height = corrector.raw_gps_data['altitude']
                
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
    gps_thread = threading.Thread(target=corrector.read_gps_data)
    gps_thread.daemon = True
    gps_thread.start()
    
    status_thread = threading.Thread(target=corrector.print_gps_status)
    status_thread.daemon = True
    status_thread.start()
    
    client = NtripClient(**ntrip_config)
    ntrip_thread = threading.Thread(target=client.readLoop)
    ntrip_thread.daemon = True
    ntrip_thread.start()
    
    print(f"System started:")
    print(f"- GPS: {corrector.gps_port}")
    print(f"- NTRIP: {ntrip_config['caster']}")
    print(f"- RTKLIB temp: {corrector.temp_dir}")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping system...")
        corrector.stop()

if __name__ == '__main__':
    main()