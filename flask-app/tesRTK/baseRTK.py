# ========================================
# RTK BASE STATION - LC29H + ESP32
# ========================================

# main.py - Upload ke ESP32 dengan MicroPython
import machine
import network
import socket
import time
import _thread
from machine import UART, Pin

# ========================================
# KONFIGURASI HARDWARE
# ========================================
# Koneksi LC29H ke ESP32:
# LC29H TX -> ESP32 GPIO16 (RX2)
# LC29H RX -> ESP32 GPIO17 (TX2) 
# LC29H VCC -> ESP32 3.3V
# LC29H GND -> ESP32 GND

# ========================================
# KONFIGURASI BASE STATION
# ========================================
# WiFi Settings
WIFI_SSID = "REALME"
WIFI_PASSWORD = "12345678"

# Base Station Position (koordinat yang sudah diketahui akurat)
# Format: latitude, longitude, altitude (WGS84)
BASE_LAT = -6.123456  # Ganti dengan koordinat base Anda
BASE_LON = 106.789012  # Ganti dengan koordinat base Anda  
BASE_ALT = 100.0      # Ganti dengan altitude base Anda (meter)

# NTRIP Caster Settings (optional - untuk publish ke internet)
NTRIP_HOST = "ntrip-caster.com"
NTRIP_PORT = 2101
NTRIP_MOUNT = "BASE001"
NTRIP_USER = "basestation"
NTRIP_PASS = "password"

# Server Settings (untuk direct connection)
SERVER_PORT = 2101

# ========================================
# SETUP HARDWARE
# ========================================
# UART untuk komunikasi dengan LC29H
uart_gps = UART(2, baudrate=460800, tx=17, rx=16, timeout=1000)

# LED indicator
led = Pin(2, Pin.OUT)  # ESP32 built-in LED

# Status variables
base_mode_active = False
rtcm_data_buffer = bytearray()
client_connections = []

def blink_led(times=1, delay=0.1):
    """Blink LED untuk status indicator"""
    for _ in range(times):
        led.on()
        time.sleep(delay)
        led.off()
        time.sleep(delay)

# ========================================
# WIFI CONNECTION
# ========================================
def connect_wifi():
    """Connect ke WiFi"""
    print("Connecting to WiFi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # Wait for connection
        timeout = 20
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1
        
    if wlan.isconnected():
        print(f"\nWiFi connected! IP: {wlan.ifconfig()[0]}")
        blink_led(3)  # 3x blink success
        return wlan.ifconfig()[0]
    else:
        print("\nWiFi connection failed!")
        return None

# ========================================
# LC29H CONFIGURATION
# ========================================
def send_command(command, wait_response=True):
    """Send command ke LC29H dan tunggu response"""
    uart_gps.write(command.encode() + b'\r\n')
    
    if wait_response:
        time.sleep(0.5)
        response = uart_gps.read()
        if response:
            return response.decode('utf-8', errors='ignore')
    return None

def setup_base_station():
    """Configure LC29H sebagai base station"""
    global base_mode_active
    
    print("Configuring LC29H as RTK Base Station...")
    
    # 1. Reset ke default
    send_command("$PQTMRESTOREPAR*13")
    time.sleep(2)
    
    # 2. Set base station mode dengan koordinat yang diketahui
    # Format: $PQTMCFGRCVR,<mode>,<lat>,<lon>,<alt>
    # Mode: 1=Survey, 2=Fixed position
    base_cmd = f"$PQTMCFGRCVR,2,{BASE_LAT},{BASE_LON},{BASE_ALT}"
    
    # Calculate checksum
    checksum = 0
    for c in base_cmd[1:]:  # Skip $
        checksum ^= ord(c)
    
    full_cmd = f"{base_cmd}*{checksum:02X}"
    print(f"Setting base position: {full_cmd}")
    
    response = send_command(full_cmd)
    print(f"Base config response: {response}")
    
    # 3. Enable RTCM output
    rtcm_commands = [
        "$PQTMCFGMSGRATE,1005,1",  # Stationary RTK reference station ARP
        "$PQTMCFGMSGRATE,1077,1",  # GPS MSM7
        "$PQTMCFGMSGRATE,1087,1",  # GLONASS MSM7  
        "$PQTMCFGMSGRATE,1097,1",  # Galileo MSM7
        "$PQTMCFGMSGRATE,1127,1",  # BeiDou MSM7
        "$PQTMCFGMSGRATE,1230,5",  # GLONASS code-phase biases
    ]
    
    for cmd in rtcm_commands:
        # Calculate checksum untuk setiap command
        checksum = 0
        for c in cmd[1:]:
            checksum ^= ord(c)
        full_cmd = f"{cmd}*{checksum:02X}"
        
        print(f"Enabling: {full_cmd}")
        send_command(full_cmd)
        time.sleep(0.1)
    
    # 4. Save configuration
    send_command("$PQTMSAVEPAR*5A")
    
    print("Base station configuration complete!")
    base_mode_active = True
    blink_led(5)  # 5x blink untuk success

# ========================================
# RTCM DATA HANDLING
# ========================================
def read_rtcm_data():
    """Read RTCM data dari LC29H"""
    global rtcm_data_buffer
    
    while True:
        if uart_gps.any():
            data = uart_gps.read()
            if data:
                # RTCM messages start with 0xD3
                for byte in data:
                    if byte == 0xD3:  # RTCM preamble
                        # Start of new RTCM message
                        if rtcm_data_buffer:
                            # Process previous message
                            process_rtcm_message(bytes(rtcm_data_buffer))
                        rtcm_data_buffer = bytearray([byte])
                    elif rtcm_data_buffer:
                        rtcm_data_buffer.append(byte)
                        
                        # Check if we have complete message
                        if len(rtcm_data_buffer) >= 3:
                            # Get message length from header
                            msg_len = ((rtcm_data_buffer[1] & 0x03) << 8) | rtcm_data_buffer[2]
                            total_len = msg_len + 6  # Header(3) + Data(msg_len) + CRC(3)
                            
                            if len(rtcm_data_buffer) >= total_len:
                                # Complete message received
                                process_rtcm_message(bytes(rtcm_data_buffer[:total_len]))
                                rtcm_data_buffer = rtcm_data_buffer[total_len:]
        
        time.sleep(0.01)  # Small delay

def process_rtcm_message(rtcm_msg):
    """Process dan distribute RTCM message ke clients"""
    global client_connections
    
    if len(rtcm_msg) < 6:
        return
    
    # Extract message type
    msg_type = (rtcm_msg[3] << 4) | (rtcm_msg[4] >> 4)
    
    print(f"RTCM {msg_type}: {len(rtcm_msg)} bytes")
    
    # Send ke semua connected clients
    dead_clients = []
    for client in client_connections:
        try:
            client.send(rtcm_msg)
        except:
            dead_clients.append(client)
    
    # Remove dead connections
    for client in dead_clients:
        client_connections.remove(client)
        try:
            client.close()
        except:
            pass

# ========================================
# TCP SERVER
# ========================================
def handle_client(client_socket, addr):
    """Handle individual client connection"""
    print(f"Client connected from {addr}")
    client_connections.append(client_socket)
    
    try:
        # Send welcome message (optional)
        welcome = f"RTK Base Station\nPosition: {BASE_LAT}, {BASE_LON}, {BASE_ALT}\n"
        client_socket.send(welcome.encode())
        
        # Keep connection alive
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            # Echo back atau process request dari client
            
    except Exception as e:
        print(f"Client error: {e}")
    finally:
        if client_socket in client_connections:
            client_connections.remove(client_socket)
        client_socket.close()
        print(f"Client {addr} disconnected")

def start_tcp_server():
    """Start TCP server untuk RTCM distribution"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', SERVER_PORT))
    server_socket.listen(5)
    
    print(f"RTK Base Server listening on port {SERVER_PORT}")
    
    while True:
        try:
            client_socket, addr = server_socket.accept()
            # Handle client di thread terpisah
            _thread.start_new_thread(handle_client, (client_socket, addr))
        except Exception as e:
            print(f"Server error: {e}")

# ========================================
# STATUS MONITORING
# ========================================
def monitor_status():
    """Monitor status base station"""
    while True:
        print(f"Base Station Status:")
        print(f"  Mode: {'Active' if base_mode_active else 'Inactive'}")
        print(f"  Connected clients: {len(client_connections)}")
        print(f"  Position: {BASE_LAT}, {BASE_LON}, {BASE_ALT}")
        print(f"  Memory free: {machine.mem_free()} bytes")
        print("-" * 40)
        
        # Blink LED untuk show alive
        blink_led(1)
        
        time.sleep(30)  # Status update setiap 30 detik

# ========================================
# MAIN PROGRAM
# ========================================
def main():
    print("=" * 50)
    print("RTK BASE STATION - LC29H + ESP32")
    print("=" * 50)
    
    # 1. Connect WiFi
    ip = connect_wifi()
    if not ip:
        print("Cannot start without WiFi!")
        return
    
    # 2. Setup LC29H as base station
    setup_base_station()
    
    # 3. Start RTCM reader thread
    _thread.start_new_thread(read_rtcm_data, ())
    
    # 4. Start status monitor thread  
    _thread.start_new_thread(monitor_status, ())
    
    # 5. Start TCP server (main thread)
    print(f"Base station ready!")
    print(f"Rovers can connect to: {ip}:{SERVER_PORT}")
    print("=" * 50)
    
    start_tcp_server()

# Start the base station
if __name__ == "__main__":
    main()