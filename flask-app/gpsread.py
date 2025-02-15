import serial
import time
import pynmea2
from typing import Optional, Tuple

class GPSReader:
    def __init__(self, port: str, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect()

    def connect(self):
        try:
            if self.ser is None or not self.ser.is_open:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=0.5)
                print(f"Connected to GPS on {self.port}")
        except serial.SerialException as e:
            print(f"Failed to connect to GPS: {e}")
            self.ser = None

    def read(self) -> Optional[Tuple[float, float]]:
        """
        Membaca data GPS
        Returns:
            Tuple[float, float]: (latitude, longitude) jika berhasil
            None: jika gagal
        """
        try:
            if self.ser is None or not self.ser.is_open:
                self.connect()
                if self.ser is None:
                    return None

            newdata = self.ser.readline()
            
            if newdata:
                newdata = newdata.decode('ascii', errors='ignore')
                
                if newdata.startswith("$GPRMC"):
                    try:
                        newmsg = pynmea2.parse(newdata)
                        lat = newmsg.latitude
                        lng = newmsg.longitude
                        print(f"Latitude={lat} and Longitude={lng}")
                        return lat, lng
                    except pynmea2.nmea.ParseError as e:
                        print(f"Failed to parse NMEA sentence: {e}")
                        return None
                else:
                    print("Waiting for valid NMEA data...")
                    return None
            else:
                print("No data received")
                return None

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.ser = None
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None

    def close(self):
        """Menutup koneksi serial"""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("GPS connection closed")

if __name__ == "__main__":
    gps = GPSReader(port="/dev/ttyACM0")
    try:
        while True:
            coords = gps.read()
            if coords:
                lat, lng = coords
                print(f"Valid coordinates received: {lat}, {lng}")
            time.sleep(0.2)  # 5Hz update rate
    except KeyboardInterrupt:
        print("\nStopping GPS reader...")
    finally:
        gps.close()