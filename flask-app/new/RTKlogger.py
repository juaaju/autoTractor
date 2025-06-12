#!/usr/bin/env python3
"""
Complete RTK System - Send RTCM + Monitor Status + CSV Logging
Optimized for reduced lagging and buffer issues
"""

import time
import threading
import serial
import csv
import os
from io import BytesIO
from ntripclient import NtripClient
import pynmea2
from datetime import datetime

class CompleteRTKSystem:
    def __init__(self, gps_port='/dev/ttyUSB1', gps_baudrate=115200):
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self.gps_serial = None
        
        # RTK Status
        self.rtcm_count = 0
        self.last_rtcm_time = 0
        self.current_position = {}
        
        # Optimized buffer - smaller size untuk reduce lag
        self.nmea_buffer = ""
        self.max_buffer_size = 1024  # Limit buffer size
        
        # Threading
        self.running = False
        self.data_lock = threading.Lock()
        
        # CSV Logging
        self.setup_csv_logging()
        
        print("🚀 Complete RTK System - RTCM Sender + Status Monitor + CSV Logging")
        print(f"GPS Port: {gps_port} @ {gps_baudrate}")
        print(f"📄 CSV Log: {self.csv_filename}")
    
    def setup_csv_logging(self):
        """Setup CSV logging"""
        # Create logs directory
        if not os.path.exists('RTKlogs'):
            os.makedirs('RTKlogs')
        
        # CSV filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'RTKlogs/rtk_log_{timestamp}.csv'
        
        # CSV headers
        self.csv_headers = [
            'timestamp', 'gps_time', 'latitude', 'longitude', 'altitude',
            'quality', 'quality_desc', 'satellites', 'hdop', 'rtcm_age',
            'rtcm_count', 'rtcm_last_sent', 'diff_station'
        ]
        
        # Write headers
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.csv_headers)
        
        print(f"✓ CSV logging initialized: {self.csv_filename}")
    
    def log_to_csv(self, position_data):
        """Log position data to CSV"""
        try:
            quality_desc = {
                0: "Invalid", 1: "GPS", 2: "DGPS", 3: "PPS",
                4: "RTK_Fixed", 5: "RTK_Float", 6: "Estimated"
            }.get(position_data.get('quality', 0), "Unknown")
            
            rtcm_last_sent = time.time() - self.last_rtcm_time if self.last_rtcm_time > 0 else 999
            
            row_data = [
                datetime.now().isoformat(),
                position_data.get('time', ''),
                position_data.get('latitude', 0),
                position_data.get('longitude', 0),
                position_data.get('altitude', 0),
                position_data.get('quality', 0),
                quality_desc,
                position_data.get('satellites', 0),
                position_data.get('hdop', 99.0),
                position_data.get('rtcm_age', 0),
                self.rtcm_count,
                rtcm_last_sent,
                position_data.get('diff_station', '')
            ]
            
            with open(self.csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row_data)
                
        except Exception as e:
            print(f"❌ CSV logging error: {e}")
    
    def connect_gps(self):
        """Connect to GPS receiver"""
        try:
            self.gps_serial = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=0.05,  # Reduced timeout untuk faster response
                rtscts=False,
                dsrdtr=False
            )
            # Clear input buffer
            self.gps_serial.reset_input_buffer()
            print(f"✓ GPS connected")
            return True
        except Exception as e:
            print(f"❌ GPS connection failed: {e}")
            return False
    
    def read_gps_and_monitor(self):
        """Read GPS data and monitor RTK status - Optimized"""
        if not self.connect_gps():
            return
        
        print("📡 Reading GPS data and monitoring RTK...")
        
        while self.running:
            try:
                # Read available data
                if self.gps_serial.in_waiting > 0:
                    # Read in chunks untuk avoid buffer overflow
                    chunk_size = min(512, self.gps_serial.in_waiting)
                    data = self.gps_serial.read(chunk_size)
                    
                    try:
                        text_data = data.decode('ascii', errors='ignore')
                        self.nmea_buffer += text_data
                        
                        # Prevent buffer dari terlalu besar (anti-lag)
                        if len(self.nmea_buffer) > self.max_buffer_size:
                            # Keep only last part of buffer
                            lines = self.nmea_buffer.split('\n')
                            self.nmea_buffer = '\n'.join(lines[-5:])  # Keep last 5 lines
                        
                        self.process_nmea_buffer()
                    except:
                        pass
                
                # Faster loop untuk reduce lag
                time.sleep(0.005)  # 5ms instead of 10ms
                
            except Exception as e:
                print(f"❌ GPS read error: {e}")
                time.sleep(0.5)
        
        if self.gps_serial:
            self.gps_serial.close()
    
    def process_nmea_buffer(self):
        """Process NMEA buffer - Optimized"""
        if '\n' not in self.nmea_buffer:
            return
            
        lines = self.nmea_buffer.split('\n')
        self.nmea_buffer = lines[-1]  # Keep incomplete line
        
        # Process only GGA sentences untuk efficiency
        for line in lines[:-1]:
            line = line.strip()
            if (line.startswith('$GNGGA') or line.startswith('$GPGGA')) and '*' in line:
                self.process_gga_sentence(line)
                break  # Process only first GGA found untuk avoid lag
    
    def process_gga_sentence(self, sentence):
        """Process GGA sentence and extract RTK info"""
        try:
            # Try pynmea2 first
            msg = pynmea2.parse(sentence)
            
            if msg.latitude and msg.longitude:
                with self.data_lock:
                    self.current_position = {
                        'timestamp': time.time(),
                        'time': str(msg.timestamp) if msg.timestamp else '',
                        'latitude': float(msg.latitude),
                        'longitude': float(msg.longitude),
                        'altitude': float(msg.altitude) if msg.altitude else 0.0,
                        'quality': int(msg.gps_qual) if msg.gps_qual else 0,
                        'satellites': int(msg.num_sats) if msg.num_sats else 0,
                        'hdop': float(msg.horizontal_dil) if msg.horizontal_dil else 99.0,
                        'rtcm_age': float(msg.age_gps_data) if hasattr(msg, 'age_gps_data') and msg.age_gps_data and msg.age_gps_data != '' else 0,
                        'diff_station': msg.diff_ref_station_id if hasattr(msg, 'diff_ref_station_id') and msg.diff_ref_station_id else '',
                        'raw_sentence': sentence
                    }
                    
                    # Log to CSV
                    self.log_to_csv(self.current_position)
                        
        except Exception as e:
            # Try manual parsing
            self.manual_parse_gga(sentence)
    
    def manual_parse_gga(self, sentence):
        """Manual parsing for problematic GGA sentences"""
        try:
            sentence_part = sentence.split('*')[0]
            fields = sentence_part.split(',')
            
            if len(fields) >= 15:
                with self.data_lock:
                    self.current_position = {
                        'timestamp': time.time(),
                        'time': fields[1],
                        'latitude': self.parse_coordinate(fields[2], fields[3]),
                        'longitude': self.parse_coordinate(fields[4], fields[5]),
                        'altitude': float(fields[9]) if fields[9] else 0.0,
                        'quality': int(fields[6]) if fields[6] else 0,
                        'satellites': int(fields[7]) if fields[7] else 0,
                        'hdop': float(fields[8]) if fields[8] else 99.0,
                        'rtcm_age': float(fields[13]) if fields[13] else 0,
                        'diff_station': fields[14] if len(fields) > 14 and fields[14] else '',
                        'raw_sentence': sentence
                    }
                    
                    # Log to CSV
                    self.log_to_csv(self.current_position)
                        
        except Exception as e:
            pass  # Silent fail for manual parsing
    
    def parse_coordinate(self, coord_str, direction):
        """Parse coordinate from NMEA format"""
        try:
            if not coord_str or coord_str == '':
                return 0.0
            
            degrees = int(float(coord_str) // 100)
            minutes = float(coord_str) % 100
            decimal = degrees + minutes / 60.0
            
            if direction in ['S', 'W']:
                decimal = -decimal
                
            return decimal
        except:
            return 0.0
    
    def send_rtcm_to_gps(self, rtcm_data):
        """Send RTCM data to GPS - Optimized"""
        if not self.gps_serial or not self.gps_serial.is_open:
            return False
        
        try:
            # Clear output buffer first untuk avoid accumulation
            self.gps_serial.reset_output_buffer()
            self.gps_serial.write(rtcm_data)
            self.gps_serial.flush()
            
            with self.data_lock:
                self.rtcm_count += 1
                self.last_rtcm_time = time.time()
            
            return True
        except Exception as e:
            print(f"❌ RTCM send error: {e}")
            return False
    
    def display_status_periodically(self):
        """Display RTK status periodically"""
        last_display_time = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                # Update display every 1.5 seconds untuk reduce CPU load
                if current_time - last_display_time >= 1.5:
                    with self.data_lock:
                        if self.current_position:
                            self.display_rtk_status()
                        else:
                            self.display_waiting_status()
                    
                    last_display_time = current_time
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"❌ Display error: {e}")
                time.sleep(1)
    
    def display_waiting_status(self):
        """Display waiting status"""
        print("\033[H\033[J", end="")  # Clear screen
        
        print("⏳ WAITING FOR GPS DATA")
        print("=" * 50)
        print(f"⏰ Time: {time.strftime('%H:%M:%S')}")
        print(f"📡 RTCM packets sent: {self.rtcm_count}")
        
        rtcm_age = time.time() - self.last_rtcm_time if self.last_rtcm_time > 0 else 999
        print(f"📡 Last RTCM: {rtcm_age:.1f}s ago")
        print(f"📄 CSV Log: {self.csv_filename}")
        print()
        print("Waiting for GPS NMEA data...")
        print("Press Ctrl+C to stop...")
    
    def display_rtk_status(self):
        """Display detailed RTK status"""
        pos = self.current_position
        quality = pos['quality']
        
        quality_info = {
            0: {"status": "❌ Invalid", "accuracy": "No fix", "color": "🔴"},
            1: {"status": "📡 GPS Standard", "accuracy": "3-5 meters", "color": "🟡"},
            2: {"status": "🔸 DGPS", "accuracy": "1-3 meters", "color": "🟠"},
            3: {"status": "🔶 PPS", "accuracy": "Variable", "color": "🟠"},
            4: {"status": "🎯 RTK Fixed", "accuracy": "1-2 cm", "color": "🟢"},
            5: {"status": "🔹 RTK Float", "accuracy": "10-50 cm", "color": "🔵"},
            6: {"status": "⚠️ Estimated", "accuracy": "Variable", "color": "🟡"},
        }
        
        info = quality_info.get(quality, {"status": f"Unknown({quality})", "accuracy": "Unknown", "color": "⚫"})
        
        # Clear screen
        print("\033[H\033[J", end="")
        
        print("🌍 COMPLETE RTK SYSTEM STATUS")
        print("=" * 60)
        print(f"⏰ Time: {time.strftime('%H:%M:%S')} | GPS: {pos['time']}")
        print()
        
        # RTK Status
        print(f"{info['color']} GPS STATUS: {info['status']}")
        print(f"📐 Accuracy: {info['accuracy']}")
        print()
        
        # Position
        print("📍 CURRENT POSITION:")
        print(f"   Lat: {pos['latitude']:.8f}°")
        print(f"   Lon: {pos['longitude']:.8f}°")
        print(f"   Alt: {pos['altitude']:.2f}m")
        print()
        
        # GPS Quality
        print("📊 GPS METRICS:")
        print(f"   Quality: {quality}")
        print(f"   Satellites: {pos['satellites']}")
        print(f"   HDOP: {pos['hdop']:.2f}")
        print()
        
        # RTCM Status
        rtcm_age = time.time() - self.last_rtcm_time if self.last_rtcm_time > 0 else 999
        print("📡 RTCM CORRECTIONS:")
        print(f"   Packets sent: {self.rtcm_count}")
        print(f"   Last sent: {rtcm_age:.1f}s ago")
        print(f"   Status: {'✅ Active' if rtcm_age < 10 else '❌ Stale'}")
        
        if pos['rtcm_age'] > 0:
            print(f"   GPS reports age: {pos['rtcm_age']:.1f}s")
        if pos['diff_station']:
            print(f"   Station ID: {pos['diff_station']}")
        print()
        
        # CSV Logging Status
        print("📄 CSV LOGGING:")
        print(f"   File: {os.path.basename(self.csv_filename)}")
        print(f"   Size: {os.path.getsize(self.csv_filename)/1024:.1f} KB" if os.path.exists(self.csv_filename) else "   Size: 0 KB")
        print()
        
        # RTK Analysis
        if quality >= 4:
            print("🎉 RTK ACTIVE!")
            if quality == 4:
                print("   🎯 RTK Fixed - Centimeter precision!")
            elif quality == 5:
                print("   🔹 RTK Float - Decimeter precision")
        elif quality == 2:
            print("🔸 DGPS Active - Meter precision")
        elif quality == 1:
            print("⏳ Standard GPS - Waiting for RTK...")
            if self.rtcm_count == 0:
                print("   ❌ No RTCM sent yet")
            elif rtcm_age > 30:
                print("   ⚠️ RTCM too old")
            else:
                print("   📡 RTCM flowing, GPS processing...")
        
        print()
        print("Press Ctrl+C to stop...")
    
    def start(self):
        """Start the complete system"""
        self.running = True
        
        # Start GPS reading thread
        gps_thread = threading.Thread(target=self.read_gps_and_monitor)
        gps_thread.daemon = True
        gps_thread.start()
        
        # Start display thread
        display_thread = threading.Thread(target=self.display_status_periodically)
        display_thread.daemon = True
        display_thread.start()
        
        return True
    
    def stop(self):
        """Stop the system"""
        self.running = False
        print(f"\n📄 Data logged to: {self.csv_filename}")

def main():
    print("🚀 COMPLETE RTK SYSTEM WITH CSV LOGGING")
    print("Sends RTCM + Monitors GPS Status + Logs Data")
    print("=" * 50)
    
    # Initialize system
    rtk_system = CompleteRTKSystem(gps_port='/dev/ttyUSB1', gps_baudrate=115200)
    
    # NTRIP configuration
    ntrip_config = {
        'user': 'toor:toor123456',
        'caster': 'nrtk.big.go.id',
        'port': 2001,
        'mountpoint': '/Nearest-rtcm3',
        'lat': -7.2839114,
        'lon': 112.7961259,
        'height': 10.0,
        'verbose': False,
        'out': BytesIO()
    }
    
    # Modify NtripClient untuk forward RTCM
    def new_receive_data(self):
        lastGGATime = time.time()
        
        try:
            while True:
                current_time = time.time()
                
                # Send GGA to NTRIP
                if current_time - lastGGATime > 10:
                    # Use current GPS position if available
                    if rtk_system.current_position:
                        self.setPosition(
                            rtk_system.current_position['latitude'],
                            rtk_system.current_position['longitude']
                        )
                        self.height = rtk_system.current_position['altitude']
                    else:
                        self.setPosition(ntrip_config['lat'], ntrip_config['lon'])
                        self.height = ntrip_config['height']
                    
                    self.socket.send(self.getGGAString().encode())
                    lastGGATime = current_time
                
                self.socket.settimeout(1.0)
                try:
                    data = self.socket.recv(self.buffer)
                    if not data:
                        break
                    
                    # Forward RTCM to GPS
                    rtk_system.send_rtcm_to_gps(data)
                    
                except Exception:
                    continue
                    
        except KeyboardInterrupt:
            print("NTRIP stopped")
        finally:
            if self.socket:
                self.socket.close()
    
    NtripClient.receiveData = new_receive_data
    
    # Start system
    if rtk_system.start():
        print("✓ RTK system started")
        print("🔄 Connecting to NTRIP...")
        
        # Start NTRIP
        client = NtripClient(**ntrip_config)
        ntrip_thread = threading.Thread(target=client.readLoop)
        ntrip_thread.daemon = True
        ntrip_thread.start()
        
        print("✓ Complete system running")
        print("📡 RTCM corrections flowing to GPS")
        print("🎯 Real-time RTK status monitoring")
        print("📄 CSV logging active")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    
    rtk_system.stop()

if __name__ == '__main__':
    main()
