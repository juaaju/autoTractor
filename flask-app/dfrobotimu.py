import serial
import struct
import time
import os
import threading

# Konfigurasi
serial_port = '/dev/ttyTHS0'  # Sesuaikan dengan port serial yang benar
baud_rate = 9600
RANGE_ACCEL = 2  # Range akselerometer ±2g
RANGE_GYRO = 250  # Range giroskop ±250°/s

class IMU_WT61PCTTL:
    """
    Kelas sederhana untuk mengambil data dari sensor IMU WT61PCTTL
    """
    
    def __init__(self, port='/dev/ttyTHS0', baudrate=9600, accel_range=2, gyro_range=250):
        """
        Inisialisasi IMU
        """
        self.port = port
        self.baudrate = baudrate
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.ser = None
        self.connected = False
        
        # Bias default (nol)
        self.accel_bias = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_bias = {'x': 0, 'y': 0, 'z': 0}
        
        # Connect saat inisialisasi
        self.connect()
    
    def calibrate_bias(self, samples=100, delay=0.01):
        if not self.connected:
            if not self.connect():
                return None
        
        print(f"Kalibrasi bias IMU sedang berjalan.")
        
        # Akumulator
        accel_x_sum = 0
        accel_y_sum = 0
        accel_z_sum = 0
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0
        
        # Sampel yang valid
        valid_samples = 0
        
        for _ in range(samples):
            data = self.get_all_data()
            if data:
                accel_x_sum += data['accel']['x']
                accel_y_sum += data['accel']['y']
                accel_z_sum += data['accel']['z'] - 1.0  # Kurangi 1g untuk kompensasi gravitasi
                gyro_x_sum += data['gyro']['x']
                gyro_y_sum += data['gyro']['y']
                gyro_z_sum += data['gyro']['z']
                valid_samples += 1
            time.sleep(delay)
        
        if valid_samples > 0:
            # Hitung rata-rata sebagai bias
            self.accel_bias = {
                'x': accel_x_sum / valid_samples,
                'y': accel_y_sum / valid_samples,
                'z': accel_z_sum / valid_samples
            }
                # Simpan juga dalam unit m/s²
            self.accel_bias_ms2 = {
                'x': self.accel_bias['x'] * 9.81,
                'y': self.accel_bias['y'] * 9.81,
                'z': self.accel_bias['z'] * 9.81
            }
            
            self.gyro_bias = {
                'x': gyro_x_sum / valid_samples,
                'y': gyro_y_sum / valid_samples,
                'z': gyro_z_sum / valid_samples
            }
            
            print(f"Kalibrasi selesai dengan {valid_samples} sampel valid")
            print(f"Bias akselerometer: x={self.accel_bias['x']:.6f}g, y={self.accel_bias['y']:.6f}g, z={self.accel_bias['z']:.6f}g")
            print(f"Bias giroskop: x={self.gyro_bias['x']:.6f}°/s, y={self.gyro_bias['y']:.6f}°/s, z={self.gyro_bias['z']:.6f}°/s")
            
            return {
                'accel': self.accel_bias,
                'gyro': self.gyro_bias
            }
        else:
            print("Kalibrasi gagal: tidak ada data valid")
            return None

    def connect(self):
        """Membuka koneksi serial"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.connected = True
            return True
        except Exception as e:
            print(f"Error membuka port serial: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Menutup koneksi serial"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
    
    def get_all_data(self, unit='g'):
        if not self.connected:
            if not self.connect():
                return None
        
        # Data default
        accel_data = {'x': 0, 'y': 0, 'z': 0, 'temp': 0}
        gyro_data = {'x': 0, 'y': 0, 'z': 0, 'temp': 0}
        angle_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
        
        # Baca data maksimal selama 1 detik
        start_time = time.time()
        data_complete = False
        
        # Flags untuk melacak tipe data yang sudah dibaca
        got_accel = False
        got_gyro = False
        got_angle = False
        
        while time.time() - start_time < 1.0 and not (got_accel and got_gyro and got_angle):
            # Mencari header paket
            header = self.ser.read(1)
            if header != b'\x55':
                continue
            
            # Membaca tipe data
            data_type = self.ser.read(1)
            
            # Membaca 8 byte data
            if data_type in [b'\x51', b'\x52', b'\x53']:
                data = self.ser.read(8)
                
                if data_type == b'\x51':  # Akselerometer
                    ax, ay, az, temperature = struct.unpack('<hhhh', data)
                    ax = ax / 32768.0 * self.accel_range
                    ay = ay / 32768.0 * self.accel_range
                    az = az / 32768.0 * self.accel_range
                    temperature = temperature / 340.0 + 36.25
                    
                    accel_data = {'x': ax, 'y': ay, 'z': az, 'temp': temperature}
                    got_accel = True
                    
                elif data_type == b'\x52':  # Giroskop
                    gx, gy, gz, temperature = struct.unpack('<hhhh', data)
                    gx = gx / 32768.0 * self.gyro_range
                    gy = gy / 32768.0 * self.gyro_range
                    gz = gz / 32768.0 * self.gyro_range
                    temperature = temperature / 340.0 + 36.25
                    
                    gyro_data = {'x': gx, 'y': gy, 'z': gz, 'temp': temperature}
                    got_gyro = True
                    
                elif data_type == b'\x53':  # Sudut
                    roll, pitch, yaw, temperature = struct.unpack('<hhhh', data)
                    roll = roll / 32768.0 * 180
                    pitch = pitch / 32768.0 * 180
                    yaw = yaw / 32768.0 * 180
                    
                    angle_data = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
                    got_angle = True
            
            # Jika sudah dapat semua jenis data, keluar dari loop
            if got_accel and got_gyro and got_angle:
                data_complete = True
                break
        
        # Koreksi bias
        gyro_data['x'] -= self.gyro_bias['x']
        gyro_data['y'] -= self.gyro_bias['y']
        gyro_data['z'] -= self.gyro_bias['z']
        # Koreksi bias dengan unit yang sesuai
        if unit.lower() == 'm/s2':
            # Konversi unit akselerasi ke m/s²
            accel_data['x'] = accel_data['x'] * 9.81 - self.accel_bias_ms2['x']
            accel_data['y'] = accel_data['y'] * 9.81 - self.accel_bias_ms2['y']
            accel_data['z'] = accel_data['z'] * 9.81 - self.accel_bias_ms2['z']
        else:
            # Bias dalam unit g
            accel_data['x'] -= self.accel_bias['x']
            accel_data['y'] -= self.accel_bias['y']
            accel_data['z'] -= self.accel_bias['z']

        # Gabungkan semua data
        result = {
            'accel': accel_data,
            'gyro': gyro_data,
            'angle': angle_data,
            'temp': accel_data['temp']  # Gunakan suhu dari akselerometer
        }
        
        return result
    
    def __del__(self):
        """Destructor untuk memastikan koneksi ditutup"""
        self.disconnect()

# Inisialisasi serial
def init_serial():
    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        return ser
    except Exception as e:
        print(f"Error membuka port serial: {e}")
        exit(1)

def clear_terminal():
    # Membersihkan terminal (cross-platform)
    os.system('cls' if os.name == 'nt' else 'clear')

def update_display(accel, gyro, angle):
    """Memperbarui tampilan data IMU"""
    clear_terminal()
    
    # Header
    print(f"=== IMU WT61PCTTL Data - {serial_port} @ {baud_rate} baud ===")
    print(f"Range Akselerometer: ±{RANGE_ACCEL}g | Range Giroskop: ±{RANGE_GYRO}°/s")
    print("Tekan Ctrl+C untuk keluar")
    print("=" * 50)
    # Akselerometer
    print("\033[92m>> AKSELEROMETER\033[0m")  # Warna hijau
    print(f"X: {accel['x']:+7.4f} g ({accel['x']*9.81:+7.2f} m/s²)")
    print(f"Y: {accel['y']:+7.4f} g ({accel['y']*9.81:+7.2f} m/s²)")
    print(f"Z: {accel['z']:+7.4f} g ({accel['z']*9.81:+7.2f} m/s²)")
    print(f"Temp: {accel['temp']:+7.2f}°C")
    print("-" * 50)
    
    # Giroskop
    print("\033[96m>> GIROSKOP\033[0m")  # Warna cyan
    print(f"X: {gyro['x']:+7.2f} °/s")
    print(f"Y: {gyro['y']:+7.2f} °/s")
    print(f"Z: {gyro['z']:+7.2f} °/s")
    print(f"Temp: {gyro['temp']:+7.2f}°C")
    print("-" * 50)
    
    # Sudut
    print("\033[93m>> SUDUT ORIENTASI\033[0m")  # Warna kuning
    print(f"Roll : {angle['roll']:+7.2f}°")
    print(f"Pitch: {angle['pitch']:+7.2f}°")
    print(f"Yaw  : {angle['yaw']:+7.2f}°")
    print(f"Temp : {angle['temp']:+7.2f}°C")
    
    
    print("=" * 50)

def main():
    try:
        ser = init_serial()
        print(f"Koneksi serial dibuka pada {serial_port}")
        
        # Data terakhir
        accel_data = {'x': 0, 'y': 0, 'z': 0, 'temp': 0}
        gyro_data = {'x': 0, 'y': 0, 'z': 0, 'temp': 0}
        angle_data = {'roll': 0, 'pitch': 0, 'yaw': 0, 'temp': 0}
        
        while True:
            # Mencari header paket
            header = ser.read(1)
            if header != b'\x55':
                continue
            
            # Membaca tipe data
            data_type = ser.read(1)
            
            # Membaca 8 byte data
            if data_type in [b'\x51', b'\x52', b'\x53', b'\x54']:
                data = ser.read(8)
                
                if data_type == b'\x51':  # Akselerometer
                    ax, ay, az, temperature = struct.unpack('<hhhh', data)
                    ax = ax / 32768.0 * RANGE_ACCEL
                    ay = ay / 32768.0 * RANGE_ACCEL
                    az = az / 32768.0 * RANGE_ACCEL
                    temperature = temperature / 340.0 + 36.25
                    
                    accel_data = {'x': ax, 'y': ay, 'z': az, 'temp': temperature}
                    
                elif data_type == b'\x52':  # Giroskop
                    gx, gy, gz, temperature = struct.unpack('<hhhh', data)
                    gx = gx / 32768.0 * RANGE_GYRO
                    gy = gy / 32768.0 * RANGE_GYRO
                    gz = gz / 32768.0 * RANGE_GYRO
                    temperature = temperature / 340.0 + 36.25
                    
                    gyro_data = {'x': gx, 'y': gy, 'z': gz, 'temp': temperature}
                    
                elif data_type == b'\x53':  # Sudut (Roll, Pitch)
                    roll, pitch, yaw, temperature = struct.unpack('<hhhh', data)
                    roll = roll / 32768.0 * 180
                    pitch = pitch / 32768.0 * 180
                    temperature = temperature / 340.0 + 36.25
                    yaw = yaw / 32768.0 * 180
                    
                    angle_data['yaw'] = yaw
                    angle_data['roll'] = roll
                    angle_data['pitch'] = pitch
                    # angle_data['temp'] = temperature
                    
                # elif data_type == b'\x54':  # Sudut (Yaw)
                #     _, _, yaw, temperature = struct.unpack('<hhhh', data)
                #     yaw = yaw / 32768.0 * 180
                    
                #     angle_data['yaw'] = yaw
                
                # Update tampilan
                update_display(accel_data, gyro_data, angle_data)
                
            time.sleep(0.01)  # Sedikit delay
            
    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh pengguna")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Koneksi serial ditutup")

if __name__ == "__main__":
    main()