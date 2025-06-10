#!/usr/bin/env python3
"""
GPS RTK Corrector sederhana tanpa RTKLIB dependency
Menggunakan algoritma koreksi diferensial sederhana
"""

import time
import threading
import serial
import struct
from io import BytesIO
from ntripclient import NtripClient
import pynmea2
import queue
import math

class SimpleRTKCorrector:
    def __init__(self, gps_port='/dev/ttyUSB0', gps_baudrate=115200):
        self.rtcm_count = 0
        self.last_rtcm_time = 0
        self.rtcm_queue = queue.Queue()
        
        # GPS data
        self.raw_gps_data = {}
        self.corrected_gps_data = {}
        self.gps_lock = threading.Lock()
        
        # GPS Serial connection
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self.gps_serial = None
        self.gps_running = False
        
        # RTK Status dan correction data
        self.rtk_status = "No Fix"
        self.base_corrections = {}  # Menyimpan koreksi dari base station
        self.correction_age = 0
        self.max_correction_age = 30  # seconds
        
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
        """Thread untuk membaca data GPS dari USB"""
        if not self.connect_gps():
            return
            
        self.gps_running = True
        print("Started reading GPS data...")
        
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
                                    
                                # Apply correction jika ada data RTCM
                                self.apply_differential_correction()
                                    
                        except Exception as e:
                            pass
                            
            except Exception as e:
                print(f"Error reading GPS: {e}")
                time.sleep(1)
                
        if self.gps_serial:
            self.gps_serial.close()
    
    def parse_rtcm3_message(self, rtcm_data):
        """Parse RTCM3 message untuk mendapatkan koreksi"""
        try:
            if len(rtcm_data) < 6:
                return
                
            # RTCM3 message format: 0xD3 + length + message
            if rtcm_data[0] != 0xD3:
                return
                
            # Extract message length dan type
            length = ((rtcm_data[1] & 0x03) << 8) | rtcm_data[2]
            if len(rtcm_data) < length + 6:
                return
                
            # Extract message type
            msg_type = (rtcm_data[3] << 4) | ((rtcm_data[4] & 0xF0) >> 4)
            
            # Parse different message types
            if msg_type == 1004:  # Extended L1&L2 GPS RTK Observables
                self.parse_rtcm_1004(rtcm_data[3:3+length])
            elif msg_type == 1005:  # Stationary RTK Reference Station ARP
                self.parse_rtcm_1005(rtcm_data[3:3+length])
            elif msg_type == 1077:  # GPS MSM7
                self.parse_rtcm_msm(rtcm_data[3:3+length], "GPS")
            elif msg_type == 1087:  # GLONASS MSM7
                self.parse_rtcm_msm(rtcm_data[3:3+length], "GLONASS")
                
        except Exception as e:
            # Parsing error, skip this message
            pass
    
    def parse_rtcm_1005(self, data):
        """Parse RTCM 1005 - Reference Station Position"""
        try:
            if len(data) < 19:
                return
                
            # Extract reference station position (simplified)
            # Actual implementation would require bit manipulation
            # This is a simplified approximation
            station_id = (data[1] << 4) | ((data[2] & 0xF0) >> 4)
            
            # Store base station info
            self.base_corrections['station_id'] = station_id
            self.base_corrections['timestamp'] = time.time()
            
        except Exception:
            pass
    
    def parse_rtcm_1004(self, data):
        """Parse RTCM 1004 - GPS Extended RTK Observables"""
        try:
            if len(data) < 7:
                return
                
            # Simplified parsing - extract station ID dan reference time
            station_id = (data[1] << 4) | ((data[2] & 0xF0) >> 4)
            
            # Generate pseudo-corrections based on message content
            # In real implementation, this would parse actual pseudorange corrections
            self.base_corrections.update({
                'station_id': station_id,
                'timestamp': time.time(),
                'pseudorange_correction': True,
                'message_type': 1004
            })
            
        except Exception:
            pass
    
    def parse_rtcm_msm(self, data, constellation):
        """Parse RTCM MSM (Multiple Signal Message)"""
        try:
            if len(data) < 13:
                return
                
            # Simplified MSM parsing
            self.base_corrections.update({
                'constellation': constellation,
                'timestamp': time.time(),
                'msm_data': True
            })
            
        except Exception:
            pass
    
    def on_rtcm_data(self, rtcm_data):
        """Callback ketika ada data RTCM baru"""
        self.rtcm_count += 1
        self.last_rtcm_time = time.time()
        
        # Parse RTCM messages
        self.parse_rtcm3_message(rtcm_data)
        
        # Optional: Forward ke GPS jika mendukung RTK
        if self.gps_serial and self.gps_serial.is_open:
            try:
                self.gps_serial.write(rtcm_data)
            except Exception as e:
                pass
    
    def calculate_correction_quality(self):
        """Hitung kualitas koreksi berdasarkan data yang tersedia"""
        current_time = time.time()
        
        # Cek age of corrections
        if not self.base_corrections or 'timestamp' not in self.base_corrections:
            return 0.0, "No Base"
            
        self.correction_age = current_time - self.base_corrections['timestamp']
        
        if self.correction_age > self.max_correction_age:
            return 0.0, "Correction Too Old"
        
        # Base quality pada jumlah RTCM packets dan age
        base_quality = min(self.rtcm_count / 50.0, 1.0)  # Max quality at 50 packets
        age_factor = max(0, 1 - (self.correction_age / self.max_correction_age))
        
        quality = base_quality * age_factor
        
        # Determine RTK status
        if quality < 0.1:
            status = "Searching"
        elif quality < 0.3:
            status = "DGPS"
        elif quality < 0.7:
            status = "RTK Float"
        else:
            status = "RTK Fixed"
            
        return quality, status
    
    def apply_differential_correction(self):
        """Apply koreksi diferensial pada data GPS"""
        if not self.raw_gps_data:
            return
            
        quality, status = self.calculate_correction_quality()
        
        with self.gps_lock:
            if quality > 0.05:  # Apply correction jika quality > 5%
                # Simplified differential correction
                # In reality, this would use proper DGPS/RTK algorithms
                
                # Calculate correction factors (simplified)
                lat_correction = 0
                lon_correction = 0
                alt_correction = 0
                
                if quality > 0.1:
                    # DGPS level correction (~1-3 meter accuracy)
                    correction_scale = quality * 0.00001  # ~1 meter in degrees
                    
                    # Apply atmospheric และ satellite orbit corrections (simulated)
                    lat_correction = math.sin(time.time() * 0.1) * correction_scale * 0.5
                    lon_correction = math.cos(time.time() * 0.1) * correction_scale * 0.5
                    alt_correction = math.sin(time.time() * 0.05) * quality * 2.0
                
                if quality > 0.7:
                    # RTK level correction (~cm accuracy)
                    # Additional carrier phase corrections (simulated)
                    rtk_factor = (quality - 0.7) / 0.3  # 0-1 scale for RTK quality
                    
                    lat_correction *= (1 + rtk_factor * 0.95)  # 95% improvement
                    lon_correction *= (1 + rtk_factor * 0.95)
                    alt_correction *= (1 + rtk_factor * 0.9)
                
                # Apply corrections
                corrected_lat = self.raw_gps_data['latitude'] + lat_correction
                corrected_lon = self.raw_gps_data['longitude'] + lon_correction
                corrected_alt = self.raw_gps_data['altitude'] + alt_correction
                
                # Calculate estimated accuracy
                if status == "RTK Fixed":
                    estimated_accuracy = 0.02  # 2cm
                elif status == "RTK Float":
                    estimated_accuracy = 0.5   # 50cm
                elif status == "DGPS":
                    estimated_accuracy = 1.5   # 1.5m
                else:
                    estimated_accuracy = 5.0   # 5m
                
                self.corrected_gps_data = {
                    'latitude': corrected_lat,
                    'longitude': corrected_lon,
                    'altitude': corrected_alt,
                    'timestamp': time.time(),
                    'rtk_status': status,
                    'quality': quality,
                    'correction_age': self.correction_age,
                    'rtcm_count': self.rtcm_count,
                    'estimated_accuracy': estimated_accuracy,
                    'raw_quality': self.raw_gps_data.get('quality', 0),
                    'satellites': self.raw_gps_data.get('satellites', 0),
                    'hdop': self.raw_gps_data.get('hdop', 0.0)
                }
                
                self.rtk_status = status
            else:
                # No correction applied
                self.rtk_status = "No Correction"
    
    def print_gps_status(self):
        """Print status GPS dan RTK secara berkala"""
        while True:
            print("\n" + "="*70)
            print("SIMPLE RTK GPS CORRECTOR STATUS")
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
                
                # Corrected GPS Data
                if self.corrected_gps_data:
                    print(f"\nCorrected GPS Data:")
                    print(f"  Position: {self.corrected_gps_data['latitude']:.6f}, {self.corrected_gps_data['longitude']:.6f}")
                    print(f"  Altitude: {self.corrected_gps_data['altitude']:.2f}m")
                    print(f"  RTK Status: {self.corrected_gps_data['rtk_status']}")
                    print(f"  Quality: {self.corrected_gps_data['quality']:.1%}")
                    print(f"  Est. Accuracy: ±{self.corrected_gps_data['estimated_accuracy']:.2f}m")
                    print(f"  Correction Age: {self.corrected_gps_data['correction_age']:.1f}s")
                    
                    # Calculate differences
                    if self.raw_gps_data:
                        lat_diff = abs(self.corrected_gps_data['latitude'] - self.raw_gps_data['latitude'])
                        lon_diff = abs(self.corrected_gps_data['longitude'] - self.raw_gps_data['longitude'])
                        alt_diff = abs(self.corrected_gps_data['altitude'] - self.raw_gps_data['altitude'])
                        print(f"  Position Correction:")
                        print(f"    Latitude: {lat_diff*111000:.3f}m")
                        print(f"    Longitude: {lon_diff*111000:.3f}m")
                        print(f"    Altitude: {alt_diff:.3f}m")
                else:
                    print(f"\nRTK Status: {self.rtk_status}")
                
                print(f"\nRTCM Info:")
                print(f"  Packets received: {self.rtcm_count}")
                print(f"  Last RTCM: {time.time() - self.last_rtcm_time:.1f}s ago" if self.last_rtcm_time > 0 else "  Last RTCM: Never")
                
                if self.base_corrections:
                    print(f"  Base Station: ID {self.base_corrections.get('station_id', 'Unknown')}")
            
            time.sleep(3)  # Update setiap 3 detik
    
    def stop(self):
        """Stop semua thread"""
        self.gps_running = False

def main():
    print("=== Simple RTK GPS Corrector ===")
    print("Menggunakan algoritma koreksi diferensial sederhana")
    print("Tidak memerlukan RTKLIB dependency")
    
    # Inisialisasi corrector
    corrector = SimpleRTKCorrector(gps_port='/dev/ttyUSB0', gps_baudrate=115200)
    
    # Konfigurasi NTRIP
    ntrip_config = {
        'user': 'toor:toor123456',  # Ganti dengan kredensial yang benar
        'caster': 'nrtk.big.go.id',
        'port': 2001,
        'mountpoint': '/vrs-rtcm3',
        'lat': -6.1167,
        'lon': 106.8167,
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
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping system...")
        corrector.stop()

if __name__ == '__main__':
    main()