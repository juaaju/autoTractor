import serial
import time
import pynmea2

while True:
    port = "/dev/ttyACM0"
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    
    newdata = ser.readline()  # Membaca data serial
    
    if newdata:  # Memeriksa apakah ada data yang diterima
        newdata = newdata.decode('ascii', errors='ignore')  # Decode bytes to string (ASCII)
        
        if newdata.startswith("$GPRMC"):  # Memeriksa apakah data adalah NMEA GPRMC
            try:
                newmsg = pynmea2.parse(newdata)
                lat = newmsg.latitude
                lng = newmsg.longitude
                gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
                print(gps)
            except pynmea2.nmea.ParseError as e:
                print(f"Failed to parse NMEA sentence: {e}")
        else:
            print("Invalid NMEA data")
    else:
        print("No data")
    
    time.sleep(1)  # Memberikan jeda waktu sebelum mencoba membaca data lagi
