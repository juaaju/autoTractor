#!/usr/bin/env python3
import socket
import base64
import serial
import time
import threading
import select

# ----------- NTRIP SETUP -----------
caster_host = '36.95.202.211'
caster_port = 2001
mountpoint = 'max-rtcm3'
username = 'toor'
password = 'toor123456'

# ----------- GPS SERIAL SETUP -----------
gps_serial_port = '/dev/ttyUSB0'
gps_baudrate = 115200
gps_timeout = 1

# Global variables untuk monitoring
rtk_status = "❌ No Fix"
last_position = {"lat": None, "lon": None, "quality": 0, "sats": 0, "hdop": 99.99}
rtk_fix_time = None
rtcm_received = False
last_rtcm_time = None
total_rtcm_bytes = 0

# Timeout settings
GPS_FIX_TIMEOUT = 300    # 5 menit untuk mendapat GPS fix
RTK_TIMEOUT = 900        # 15 menit untuk RTK
NO_RTCM_TIMEOUT = 120    # 2 menit tanpa RTCM data
SOCKET_TIMEOUT = 30      # 30 detik socket timeout

def parse_gga_sentence(nmea_sentence):
    """Parse NMEA GGA sentence untuk extract info RTK"""
    global rtk_status, last_position, rtk_fix_time
    
    try:
        parts = nmea_sentence.split(',')
        if len(parts) < 15:
            return False
        
        # Extract data dari GGA sentence
        quality = int(parts[6]) if parts[6] else 0
        num_sats = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 99.99
        
        # Parse koordinat jika ada
        lat = None
        lon = None
        if parts[2] and parts[4]:
            try:
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
            except:
                pass
        
        # Status mapping
        quality_status = {
            0: "❌ No Fix",
            1: "🟡 GPS Fix",
            2: "🟠 DGPS Fix", 
            4: "🎯 RTK Fixed",
            5: "🔄 RTK Float"
        }
        
        old_quality = last_position["quality"]
        new_status = quality_status.get(quality, f"❓ Quality {quality}")
        
        # Update global status
        rtk_status = new_status
        last_position.update({
            "lat": lat, "lon": lon, "quality": quality,
            "sats": num_sats, "hdop": hdop
        })
        
        # Detect RTK achievement
        if quality in [4, 5] and old_quality not in [4, 5]:
            rtk_fix_time = time.time()
            print(f"\n🎉 RTK ACHIEVED! {new_status}")
        
        return True
        
    except Exception as e:
        print(f"Error parsing GGA: {e}")
        return False

def connect_ntrip():
    """Koneksi ke NTRIP caster dengan error handling yang baik"""
    try:
        print(f"🔗 Connecting to {caster_host}:{caster_port}...")
        
        # Buat socket dengan timeout lebih panjang
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(SOCKET_TIMEOUT)
        sock.connect((caster_host, caster_port))
        
        # Authentication
        auth = base64.b64encode(f"{username}:{password}".encode()).decode()
        
        # HTTP request
        request = (
            f"GET /{mountpoint} HTTP/1.1\r\n"
            f"Host: {caster_host}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP RTK Client/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"Connection: keep-alive\r\n"  # Keep alive instead of close
            f"\r\n"
        )
        
        print("📤 Sending NTRIP request...")
        sock.send(request.encode())
        
        # Read response header with timeout
        response = b""
        start_time = time.time()
        while b"\r\n\r\n" not in response:
            if time.time() - start_time > 10:
                raise Exception("Timeout reading response header")
            
            ready = select.select([sock], [], [], 1)
            if ready[0]:
                data = sock.recv(1)
                if not data:
                    raise Exception("Connection closed while reading header")
                response += data
        
        response_str = response.decode('utf-8', errors='ignore')
        print(f"📥 Response:\n{response_str}")
        
        if "200 OK" not in response_str:
            raise Exception(f"NTRIP error: {response_str}")
        
        print("✅ NTRIP connected successfully")
        return sock
        
    except Exception as e:
        print(f"❌ NTRIP connection failed: {e}")
        return None

def connect_gps():
    """Koneksi ke GPS dengan validasi"""
    try:
        print(f"🛰️ Connecting to GPS at {gps_serial_port}...")
        gps = serial.Serial(gps_serial_port, gps_baudrate, timeout=gps_timeout)
        
        # Test GPS data
        time.sleep(2)
        test_data = ""
        start_time = time.time()
        
        while time.time() - start_time < 5:
            if gps.in_waiting > 0:
                data = gps.read_all().decode('utf-8', errors='ignore')
                test_data += data
                if '$' in test_data:
                    break
        
        if '$' in test_data:
            print(f"✅ GPS connected - NMEA data detected")
            return gps
        else:
            print(f"⚠️ GPS connected but no NMEA data")
            return gps
        
    except Exception as e:
        print(f"❌ GPS connection failed: {e}")
        return None

def monitor_gps(gps_serial, start_time):
    """Monitor GPS dengan timeout detection"""
    global last_position
    
    print("📡 Starting GPS monitoring...")
    last_update = time.time()
    
    while True:
        try:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Check GPS fix timeout
            if elapsed > GPS_FIX_TIMEOUT and last_position["quality"] == 0:
                print(f"\n⏰ GPS Fix timeout ({GPS_FIX_TIMEOUT//60} minutes)")
                print("🔧 Troubleshooting:")
                print("   - Check antenna connection")
                print("   - Move to open sky area") 
                print("   - Wait for satellite acquisition")
                break
            
            # Check RTK timeout
            if elapsed > RTK_TIMEOUT and last_position["quality"] not in [4, 5]:
                print(f"\n⏰ RTK timeout ({RTK_TIMEOUT//60} minutes)")
                print("🔧 Possible issues:")
                print("   - Distance too far from base station")
                print("   - Ionospheric conditions")
                print("   - Multipath interference")
                break
            
            # Read GPS data
            if gps_serial.in_waiting > 0:
                data = gps_serial.readline().decode('utf-8', errors='ignore').strip()
                
                if data.startswith('$') and 'GGA' in data:
                    if parse_gga_sentence(data):
                        last_update = current_time
                        
                        # Display status setiap 30 detik
                        if int(elapsed) % 30 == 0 and elapsed > 0:
                            print(f"\n📊 Status Update (T+{elapsed:.0f}s):")
                            print(f"   GPS: {rtk_status}")
                            print(f"   Satellites: {last_position['sats']}")
                            print(f"   HDOP: {last_position['hdop']:.1f}")
                            print(f"   RTCM: {total_rtcm_bytes} bytes received")
            
            # Check no GPS data timeout
            if current_time - last_update > 30:
                print(f"\n⚠️ No GPS data for {current_time - last_update:.0f} seconds")
                last_update = current_time
            
        except Exception as e:
            print(f"❌ GPS monitoring error: {e}")
            break
        
        time.sleep(0.1)

def main():
    global rtcm_received, last_rtcm_time, total_rtcm_bytes
    
    print("=" * 60)
    print("🛰️ NTRIP RTK Client with Robust Timeout Management")
    print("=" * 60)
    
    start_time = time.time()
    
    # Connect GPS
    gps = connect_gps()
    if not gps:
        return
    
    # Connect NTRIP
    sock = connect_ntrip()
    if not sock:
        gps.close()
        return
    
    # Start GPS monitoring thread
    gps_thread = threading.Thread(target=monitor_gps, args=(gps, start_time), daemon=True)
    gps_thread.start()
    
    print(f"\n🚀 Starting RTCM streaming...")
    print(f"⏰ Timeouts: GPS Fix={GPS_FIX_TIMEOUT//60}min, RTK={RTK_TIMEOUT//60}min")
    print(f"🎯 Target: Quality indicator 4 (RTK Fixed) or 5 (RTK Float)")
    print(f"Press Ctrl+C to stop\n")
    
    try:
        while True:
            # Use select for non-blocking socket read
            ready = select.select([sock], [], [], 5)  # 5 second timeout
            
            if ready[0]:
                # Socket has data
                data = sock.recv(4096)  # Larger buffer
                if not data:
                    print("❌ NTRIP connection closed by server")
                    break
                
                # Send to GPS
                gps.write(data)
                total_rtcm_bytes += len(data)
                
                if not rtcm_received:
                    rtcm_received = True
                    print("✅ RTCM data flowing to GPS")
                
                last_rtcm_time = time.time()
            
            else:
                # No data timeout
                if last_rtcm_time and time.time() - last_rtcm_time > NO_RTCM_TIMEOUT:
                    print(f"❌ No RTCM data for {NO_RTCM_TIMEOUT} seconds")
                    break
                elif not rtcm_received and time.time() - start_time > 30:
                    print("❌ No RTCM data received after 30 seconds")
                    break
            
            # Check if RTK achieved
            if last_position["quality"] in [4, 5]:
                elapsed = time.time() - rtk_fix_time if rtk_fix_time else 0
                if elapsed > 60:  # Show success message after 1 minute of stable RTK
                    print(f"\n🎉 RTK SUCCESS! Stable for {elapsed:.0f} seconds")
                    print("✅ You can now stop the client (Ctrl+C)")
            
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        print("\n🔌 Closing connections...")
        if sock:
            sock.close()
        if gps:
            gps.close()
        
        # Final summary
        print("\n" + "=" * 60)
        print("📊 FINAL SUMMARY")
        print("=" * 60)
        print(f"Final Status: {rtk_status}")
        print(f"Satellites: {last_position['sats']}")
        print(f"HDOP: {last_position['hdop']:.1f}")
        print(f"Total RTCM: {total_rtcm_bytes} bytes")
        
        if last_position["lat"] and last_position["lon"]:
            print(f"Position: {last_position['lat']:.6f}, {last_position['lon']:.6f}")
        
        if rtk_fix_time:
            print(f"✅ RTK achieved in {rtk_fix_time - start_time:.0f} seconds")
        else:
            runtime = time.time() - start_time
            print(f"❌ RTK not achieved in {runtime:.0f} seconds")
        
        print("=" * 60)

if __name__ == "__main__":
    main()