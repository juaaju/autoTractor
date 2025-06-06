#!/usr/bin/env python3
import serial
import time

def try_nmea_commands(port='/dev/ttyUSB0'):
    """
    Mencoba perintah NMEA untuk konfigurasi baudrate
    Beberapa GPS receiver bisa menerima perintah konfigurasi via NMEA
    """
    try:
        print(f"Mencoba perintah NMEA di {port}")
        ser = serial.Serial(port, 115200, timeout=3)
        time.sleep(1)
        
        # Beberapa perintah NMEA yang umum untuk konfigurasi
        nmea_commands = [
            # Standard NMEA configuration commands
            "$PMTK251,9600*17\r\n",  # MTK chipset baudrate change
            "$PCAS01,5*19\r\n",      # CAS baudrate to 9600
            "$PUBX,41,1,0007,0003,9600,0*13\r\n",  # u-blox baudrate change
            
            # Quectel specific attempts (educated guesses)
            "$PQTM,0,9600*6F\r\n",   # Quectel possible command
            "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n",  # Disable all NMEA
        ]
        
        print("Mengirim perintah NMEA...")
        for i, cmd in enumerate(nmea_commands):
            print(f"Mencoba perintah {i+1}: {cmd.strip()}")
            ser.write(cmd.encode())
            time.sleep(1)
            
            # Cek response
            if ser.in_waiting > 0:
                response = ser.read_all().decode('utf-8', errors='ignore')
                print(f"Response: {response}")
                
                # Jika ada acknowledgment, test baudrate baru
                if 'ACK' in response or 'OK' in response:
                    print("Kemungkinan berhasil, testing baudrate baru...")
                    ser.close()
                    return test_new_baudrate(port, 9600)
        
        ser.close()
        return False
        
    except Exception as e:
        print(f"Error NMEA: {e}")
        return False

def try_proprietary_commands(port='/dev/ttyUSB0'):
    """
    Mencoba perintah proprietary Quectel yang mungkin diterima di GPS port
    """
    try:
        print(f"Mencoba perintah proprietary Quectel...")
        ser = serial.Serial(port, 115200, timeout=3)
        time.sleep(1)
        
        # Perintah yang mungkin diterima oleh GPS engine
        commands = [
            # Binary commands (hex)
            b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x80\x25\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\xA2\xB5',  # u-blox style
            
            # Text commands yang mungkin
            b'$QGPS,1*28\r\n',
            b'$QGNSS,1*2B\r\n', 
            b'$QCFGgnss="baudrate",9600*XX\r\n',
            
            # AT-style tapi lewat GPS port
            b'+++\r\n',
            b'AT+QGPSCFG="baudrate",9600\r\n',
            b'AT+IPR=9600\r\n',
        ]
        
        for i, cmd in enumerate(commands):
            print(f"Mencoba command {i+1}...")
            ser.write(cmd)
            time.sleep(1)
            
            if ser.in_waiting > 0:
                response = ser.read_all().decode('utf-8', errors='ignore')
                print(f"Response: {response}")
                
                if 'OK' in response or 'ACK' in response:
                    print("Mungkin berhasil!")
                    ser.close()
                    return test_new_baudrate(port, 9600)
        
        ser.close()
        return False
        
    except Exception as e:
        print(f"Error proprietary: {e}")
        return False

def check_other_interfaces():
    """
    Cek apakah ada interface lain yang tersedia
    """
    import os
    import subprocess
    
    print("=== Checking Available Interfaces ===")
    
    # Cek semua ttyUSB
    try:
        result = subprocess.run(['ls', '/dev/ttyUSB*'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"Available ttyUSB ports: {result.stdout}")
        else:
            print("No ttyUSB ports found")
    except:
        print("Error checking ttyUSB ports")
    
    # Cek ttyACM (beberapa modem pakai ini)
    try:
        result = subprocess.run(['ls', '/dev/ttyACM*'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"Available ttyACM ports: {result.stdout}")
    except:
        print("No ttyACM ports")
    
    # Cek USB device info
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        if 'Quectel' in result.stdout:
            print("Quectel device found:")
            for line in result.stdout.split('\n'):
                if 'Quectel' in line:
                    print(f"  {line}")
    except:
        print("Cannot check USB devices")

def test_new_baudrate(port, new_baud):
    """Test koneksi dengan baudrate baru"""
    try:
        print(f"Testing baudrate {new_baud}...")
        time.sleep(2)
        
        ser = serial.Serial(port, new_baud, timeout=3)
        time.sleep(2)
        
        start_time = time.time()
        data_received = ""
        
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                data = ser.read_all().decode('utf-8', errors='ignore')
                data_received += data
                print(f"Data: {data[:50]}...")
                
                if '$GN' in data or '$GP' in data:
                    print(f"✓ NMEA data detected on baudrate {new_baud}")
                    ser.close()
                    return True
            time.sleep(0.1)
        
        if not data_received:
            print(f"No data on baudrate {new_baud}")
        
        ser.close()
        return False
        
    except Exception as e:
        print(f"Error testing baudrate: {e}")
        return False

def main():
    print("=== Quectel LC2H GPS Baudrate Configuration ===")
    print("Problem: /dev/ttyUSB1 is GPS NMEA port, not AT command port")
    print()
    
    # Check available interfaces
    check_other_interfaces()
    print()
    
    # Method 1: NMEA commands
    print("--- Method 1: NMEA Configuration Commands ---")
    success = try_nmea_commands()
    
    if not success:
        # Method 2: Proprietary commands
        print("\n--- Method 2: Proprietary Commands ---")
        success = try_proprietary_commands()
    
    if success:
        print("\n✓ Baudrate change successful!")
    else:
        print("\n❌ Baudrate change failed!")
        print("\nAlternative solutions:")
        print("1. Use Quectel QGNSS configuration software")
        print("2. Check if AT command port exists on different interface")
        print("3. Use hardware UART interface if available")
        print("4. Contact Quectel support for LC2H specific configuration")
        print("5. Check if module needs firmware update for AT commands")

if __name__ == "__main__":
    main()