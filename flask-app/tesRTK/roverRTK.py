#!/usr/bin/env python3
# RTK Rover Client - untuk konek ke base station ESP32
import socket
import serial
import time
import threading

# ========================================
# KONFIGURASI ROVER
# ========================================
# Base Station Connection
BASE_STATION_IP = "192.168.1.100"  # IP ESP32 base station
BASE_STATION_PORT = 2101

# GPS/GNSS Receiver (rover)
GPS_PORT = "/dev/ttyUSB0"  # Port GPS rover
GPS_BAUDRATE = 115200

# ========================================
# ROVER CLIENT CLASS
# ========================================
class RTKRoverClient:
    def __init__(self):
        self.base_socket = None
        self.gps_serial = None
        self.rtk_status = "No Fix"
        self.position = {"lat": None, "lon": None, "quality": 0}
        self.running = False
        
    def connect_to_base(self):
        """Connect ke base station"""
        try:
            print(f"Connecting to base station {BASE_STATION_IP}:{BASE_STATION_PORT}...")
            self.base_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.base_socket.settimeout(10)
            self.base_socket.connect((BASE_STATION_IP, BASE_STATION_PORT))
            
            # Baca welcome message
            welcome = self.base_socket.recv(1024).decode('utf-8')
            print(f"Base station says: {welcome}")
            
            print("✅ Connected to base station")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to base station: {e}")
            return False
    
    def connect_gps(self):
        """Connect ke GPS rover"""
        try:
            print(f"Connecting to GPS rover at {GPS_PORT}...")
            self.gps_serial = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1)
            
            # Test GPS data
            time.sleep(2)
            if self.gps_serial.in_waiting > 0:
                sample = self.gps_serial.read(self.gps_serial.in_waiting())
                print(f"✅ GPS rover connected - sample: {sample[:50]}...")
            else:
                print("⚠️ GPS connected but no data yet")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to GPS: {e}")
            return False
    
    def parse_gga(self, nmea_line):
        """Parse NMEA GGA untuk monitor RTK status"""
        try:
            parts = nmea_line.split(',')
            if len(parts) < 15:
                return
            
            quality = int(parts[6]) if parts[6] else 0
            num_sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.99
            
            # Parse coordinates
            lat = None
            lon = None
            if parts[2] and parts[4]:
                lat_raw = parts[2]
                lon_raw = parts[4]
                
                lat_deg = int(lat_raw[:2])
                lat_min = float(lat_raw[2:])
                lat = lat_deg + lat_min/60
                if parts[3] == 'S':
                    lat = -lat
                
                lon_deg = int(lon_raw[:3])
                lon_min = float(lon_raw[3:])
                lon = lon_deg + lon_min/60
                if parts[5] == 'W':
                    lon = -lon
            
            # Status mapping
            status_map = {
                0: "❌ No Fix",
                1: "🟡 GPS Fix",
                2: "🟠 DGPS Fix",
                4: "🎯 RTK Fixed",
                5: "🔄 RTK Float"
            }
            
            old_quality = self.position["quality"]
            self.rtk_status = status_map.get(quality, f"Unknown ({quality})")
            self.position = {
                "lat": lat, "lon": lon, "quality": quality,
                "sats": num_sats, "hdop": hdop
            }
            
            # RTK achievement notification
            if quality in [4, 5] and old_quality not in [4, 5]:
                print(f"\n🎉 RTK ACHIEVED! Status: {self.rtk_status}")
            
            return True
            
        except Exception as e:
            print(f"Error parsing GGA: {e}")
            return False
    
    def rtcm_receiver_thread(self):
        """Thread untuk receive RTCM dari base station"""
        print("📡 Starting RTCM receiver...")
        
        while self.running:
            try:
                # Receive RTCM data dari base station
                rtcm_data = self.base_socket.recv(4096)
                if not rtcm_data:
                    print("❌ Base station connection lost")
                    break
                
                # Forward ke GPS rover
                if self.gps_serial:
                    self.gps_serial.write(rtcm_data)
                
                # Debug: show RTCM message type
                if len(rtcm_data) >= 6 and rtcm_data[0] == 0xD3:
                    msg_type = (rtcm_data[3] << 4) | (rtcm_data[4] >> 4)
                    print(f"📥 RTCM {msg_type}: {len(rtcm_data)} bytes")
                
            except Exception as e:
                print(f"RTCM receiver error: {e}")
                break
    
    def gps_monitor_thread(self):
        """Thread untuk monitor GPS rover"""
        print("🛰️ Starting GPS monitor...")
        
        while self.running:
            try:
                if self.gps_serial and self.gps_serial.in_waiting > 0:
                    line = self.gps_serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line.startswith('$') and 'GGA' in line:
                        self.parse_gga(line)
                        
                        # Display status setiap GGA
                        if self.position["quality"] > 0:
                            print(f"📍 {self.rtk_status} | "
                                  f"Sats: {self.position['sats']} | "
                                  f"HDOP: {self.position['hdop']:.1f}")
                
            except Exception as e:
                print(f"GPS monitor error: {e}")
                break
            
            time.sleep(0.1)
    
    def start(self):
        """Start rover client"""
        print("=" * 60)
        print("🛰️ RTK ROVER CLIENT")
        print("=" * 60)
        
        # Connect to components
        if not self.connect_gps():
            return False
        
        if not self.connect_to_base():
            return False
        
        # Start threads
        self.running = True
        
        rtcm_thread = threading.Thread(target=self.rtcm_receiver_thread, daemon=True)
        gps_thread = threading.Thread(target=self.gps_monitor_thread, daemon=True)
        
        rtcm_thread.start()
        gps_thread.start()
        
        print("🚀 RTK Rover started!")
        print("🎯 Target: Wait for RTK Fixed (quality 4) or RTK Float (quality 5)")
        print("Press Ctrl+C to stop")
        print("-" * 60)
        
        # Main loop
        try:
            start_time = time.time()
            last_status = time.time()
            
            while self.running:
                current_time = time.time()
                
                # Status update setiap 30 detik
                if current_time - last_status > 30:
                    elapsed = current_time - start_time
                    print(f"\n📊 Status Update (T+{elapsed:.0f}s):")
                    print(f"   RTK Status: {self.rtk_status}")
                    print(f"   Satellites: {self.position['sats']}")
                    print(f"   HDOP: {self.position['hdop']:.1f}")
                    if self.position["lat"] and self.position["lon"]:
                        print(f"   Position: {self.position['lat']:.6f}, {self.position['lon']:.6f}")
                    print("-" * 40)
                    last_status = current_time
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping rover client...")
        
        # Cleanup
        self.running = False
        if self.base_socket:
            self.base_socket.close()
        if self.gps_serial:
            self.gps_serial.close()
        
        print("✅ Rover client stopped")
        return True

# ========================================
# STANDALONE USAGE
# ========================================
def main():
    # Konfigurasi bisa diubah di sini
    global BASE_STATION_IP, GPS_PORT
    
    # Input manual jika diperlukan
    import sys
    if len(sys.argv) > 1:
        BASE_STATION_IP = sys.argv[1]
    if len(sys.argv) > 2:
        GPS_PORT = sys.argv[2]
    
    print(f"Base Station: {BASE_STATION_IP}:{BASE_STATION_PORT}")
    print(f"GPS Port: {GPS_PORT}")
    
    # Start rover
    rover = RTKRoverClient()
    rover.start()

if __name__ == "__main__":
    main()