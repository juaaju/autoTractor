#!/usr/bin/env python3
"""
Complete RTK System - Send RTCM + Monitor Status
"""

import time
import threading
import serial
from io import BytesIO
from ntripclient import NtripClient
import pynmea2

class CompleteRTKSystem:
    def __init__(self, gps_port='/dev/ttyUSB1', gps_baudrate=115200):
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self.gps_serial = None
        
        # RTK Status
        self.rtcm_count = 0
        self.last_rtcm_time = 0
        self.current_position = {}
        self.position_history = []
        
        # Buffer for proper NMEA parsing
        self.nmea_buffer = ""
        
        # Threading
        self.running = False
        self.data_lock = threading.Lock()
        
        print("🚀 Complete RTK System - RTCM Sender + Status Monitor")
        print(f"GPS Port: {gps_port} @ {gps_baudrate}")
    
    def connect_gps(self):
        """Connect to GPS receiver"""
        try:
            self.gps_serial = serial.Serial(
                port=self.gps_port,
                baudrate=self.gps_baudrate,
                timeout=0.1
            )
            print(f"✓ GPS connected")
            return True
        except Exception as e:
            print(f"❌ GPS connection failed: {e}")
            return False
    
    def read_gps_and_monitor(self):
        """Read GPS data and monitor RTK status"""
        if not self.connect_gps():
            return
        
        print("📡 Reading GPS data and monitoring RTK...")
        
        while self.running:
            try:
                if self.gps_serial.in_waiting > 0:
                    data = self.gps_serial.read(self.gps_serial.in_waiting)
                    
                    # Process as text for NMEA
                    try:
                        text_data = data.decode('ascii', errors='ignore')
                        self.nmea_buffer += text_data
                        self.process_nmea_buffer()
                    except:
                        pass
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"❌ GPS read error: {e}")
                time.sleep(1)
        
        if self.gps_serial:
            self.gps_serial.close()
    
    def process_nmea_buffer(self):
        """Process NMEA buffer to extract complete sentences"""
        lines = self.nmea_buffer.split('\n')
        self.nmea_buffer = lines[-1]  # Keep incomplete line
        
        for line in lines[:-1]:
            line = line.strip()
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                if '*' in line:  # Complete sentence
                    self.process_gga_sentence(line)
    
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
                    
                    self.position_history.append(self.current_position.copy())
                    if len(self.position_history) > 10:
                        self.position_history.pop(0)
                        
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
                    
                    self.position_history.append(self.current_position.copy())
                    if len(self.position_history) > 10:
                        self.position_history.pop(0)
                        
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
        """Send RTCM data to GPS"""
        if not self.gps_serial or not self.gps_serial.is_open:
            return False
        
        try:
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
        while self.running:
            try:
                with self.data_lock:
                    if self.current_position:
                        self.display_rtk_status()
                    else:
                        self.display_waiting_status()
                
                time.sleep(2)
                
            except Exception as e:
                print(f"❌ Display error: {e}")
                time.sleep(2)
    
    def display_waiting_status(self):
        """Display waiting status"""
        print("\033[H\033[J", end="")  # Clear screen
        
        print("⏳ WAITING FOR GPS DATA")
        print("=" * 50)
        print(f"⏰ Time: {time.strftime('%H:%M:%S')}")
        print(f"📡 RTCM packets sent: {self.rtcm_count}")
        
        rtcm_age = time.time() - self.last_rtcm_time if self.last_rtcm_time > 0 else 999
        print(f"📡 Last RTCM: {rtcm_age:.1f}s ago")
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
        print(f"Raw: {pos['raw_sentence'][:80]}...")
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

def main():
    print("🚀 COMPLETE RTK SYSTEM")
    print("Sends RTCM + Monitors GPS Status")
    print("=" * 50)
    
    # Initialize system
    rtk_system = CompleteRTKSystem(gps_port='/dev/ttyUSB1', gps_baudrate=115200)
    
    # NTRIP configuration
    ntrip_config = {
        'user': 'toor:toor123456',
        'caster': 'nrtk.big.go.id',
        'port': 2001,
        'mountpoint': '/vrs-rtcm3',
        'lat': -7.2835908,
        'lon': 112.7963027,
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
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    
    rtk_system.stop()

if __name__ == '__main__':
    main()
